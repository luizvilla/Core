#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file restore_board_lines.py
## @brief Puts back the set(BOARD ...) and set(SHIELD ...) lines of zephyr/CMakeLists.txt as committed in HEAD.
##        owntech/scripts/pre_version.py rewrites (or, for no shield, removes) them on every build; every other
##        line, committed or not, is left as it is.
import os
import re
import subprocess

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PATH = os.path.join(REPO, 'zephyr', 'CMakeLists.txt')
head = subprocess.run(['git', 'show', 'HEAD:zephyr/CMakeLists.txt'], cwd=REPO, capture_output=True, text=True).stdout
want = {m.group(1): l for l in head.splitlines() for m in [re.match(r'\s*set\((BOARD|SHIELD) ', l)] if m}
lines = open(PATH).read().split('\n')
seen = set()
for i, l in enumerate(lines):
    m = re.match(r'\s*set\((BOARD|SHIELD) ', l)
    if m and m.group(1) in want:
        lines[i] = want[m.group(1)]
        seen.add(m.group(1))
if 'SHIELD' in want and 'SHIELD' not in seen:
    b = next(i for i, l in enumerate(lines) if re.match(r'\s*set\(BOARD ', l))
    lines.insert(b + 1, want['SHIELD'])
open(PATH, 'w').write('\n'.join(lines))
