#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
# Copyright (c) 2026 OWNTECH TECHNOLOGIES
"""Run the real hrtim_burst_set body against RM0440's preload handshake.

Extract only this hardware-independent function so the host test need not mock
the entire STM32 peripheral driver. The model checks register update acceptance,
not the physical PWM waveform. RM0440 rev 9, section 28.3.15, page 960:
writing BMPER blocks updates until a subsequent write to BMCMP.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
driver = (ROOT / "zephyr/modules/owntech_hrtim_driver/zephyr/src/hrtim.c").read_text()
start = driver.index("void hrtim_burst_set(")
opening = driver.index("{", start)
depth = 1
end = opening + 1
while depth:
    depth += (driver[end] == "{") - (driver[end] == "}")
    end += 1
function = driver[start:end]

model = r'''
#include "check.h"
struct BurstRegisters {
    int compare = 0, period = 0, active_compare = 0, active_period = 0;
    bool updates_blocked = false;
    void boundary() {
        if (!updates_blocked) { active_compare = compare; active_period = period; }
    }
};
static BurstRegisters registers;
static BurstRegisters* const HRTIM1 = &registers;
static void LL_HRTIM_BM_SetPeriod(BurstRegisters* r, int value) {
    r->period = value;
    r->updates_blocked = true;
    r->boundary(); // Also exercise a boundary between the two register writes.
}
static void LL_HRTIM_BM_SetCompare(BurstRegisters* r, int value) {
    r->compare = value;
    r->updates_blocked = false;
}
'''
checks = r'''
int main() {
    {
    tc::Criterion c("L0-BST-11.a", "HRTIM burst preload accepts changing skip counts and periods");
    for (const auto& pattern : {std::pair<int,int>{1, 200}, {100, 200},
                               {200, 200}, {1, 100}, {50, 100}}) {
        hrtim_burst_set(pattern.first, pattern.second);
        registers.boundary();
        c.check("update handshake released", !registers.updates_blocked);
        c.check("skip count reaches active register", registers.active_compare == pattern.first - 1);
        c.check("period reaches active register", registers.active_period == pattern.second - 1);
    }
    }
    return tc::finish();
}
'''
with tempfile.TemporaryDirectory(prefix="burst-registers-") as folder:
    source = Path(folder) / "test.cpp"
    source.write_text("#include <initializer_list>\n#include <utility>\n" + model + function + checks)
    binary = Path(folder) / "test"
    subprocess.run(shlex.split(os.environ.get("CXX", "g++")) + [
        "-std=c++20", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
        "-fno-sanitize-recover=undefined", "-I" + str(ROOT / "tests/common"),
        str(source), "-o", str(binary),
    ], check=True)
    raise SystemExit(subprocess.run([str(binary)]).returncode)
