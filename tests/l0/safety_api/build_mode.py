#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file build_mode.py
## @brief L0-API-01: development-only calls do not compile in a production build (ADR 0009, issue luizvilla/Core#22).
##
## Builds env SAFETY_MODE three times (Twist 1.4.1, about 1 min each, no board):
##   a. tests/firmware/safety_mode_prod: safety.disarm() in a production build -> must fail with the message;
##   b. tests/firmware/safety_mode_dev: the same source in development mode -> must build (control for a.);
##   c. tests/firmware/safety_mode_noselftest: run-time self-tests off in a production build -> must fail.
##
## @par Usage
##   python3 tests/l0/safety_api/build_mode.py --out DIR
import argparse
import os
import shutil
import subprocess
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
BUILD = os.path.join(REPO, '.pio', 'build', 'SAFETY_MODE')
DISARM_MSG = 'safety.disarm() exists only in development mode'
SELFTEST_MSG = 'CONFIG_OWNTECH_SAFETY_RUNTIME_SELFTEST=n is allowed only in development mode'
CASES = [
    ('a', 'safety_mode_prod', False, DISARM_MSG,
     'safety.disarm() in a production build does not compile, and the error says how to get development mode'),
    ('b', 'safety_mode_dev', True, None,
     'The same source builds with safety-mode = "development" in app.overlay'),
    ('c', 'safety_mode_noselftest', False, SELFTEST_MSG,
     'Run-time self-tests turned off (app.conf) in a production build: the build fails with the reason'),
]


def build(app, log):
    shutil.rmtree(BUILD, ignore_errors=True)   # PlatformIO does not reconfigure on overlay or app.conf edits
    env = dict(os.environ, SAFETY_MODE_APP=f'tests/firmware/{app}')
    r = subprocess.run([PIO, 'run', '-e', 'SAFETY_MODE'], cwd=REPO, env=env, capture_output=True, text=True)
    subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
    text = r.stdout + r.stderr
    open(log, 'w').write(text)
    return r.returncode, text


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for cid, app, should_build, msg, title in CASES:
        rc, text = build(app, os.path.join(a.out, f'build_{app}.txt'))
        if should_build:
            ok, measured = rc == 0, 'builds' if rc == 0 else f'exit {rc}, see build_{app}.txt'
            expected = 'builds'
        else:
            found = msg in text
            ok = rc != 0 and found
            line = next((l.strip() for l in text.splitlines() if msg in l), '')
            measured = (f'fails: {line[line.find(msg):][:110]}' if found else
                        f'exit {rc}, message not found, see build_{app}.txt')
            expected = f'build fails with "{msg[:60]}..."'
        print(f'@@\tL0-API-01.{cid}\t{"PASS" if ok else "FAIL"}\t{title}\t{expected}\t{measured}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
