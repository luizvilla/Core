#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file build_check.py
## @brief L0-ALC-11/12: every shield devicetree in zephyr/boards/shields builds with the protection allocation
##        (strict: a hardware limit with no path and no exception fails), and its generated protection table
##        matches the committed one in docs/safety/boards/ (ADR 0009, issue luizvilla/Core#17).
##
## Shields are discovered from the overlay files, so a new board is checked without editing this script. Each
## shield runs PlatformIO env SAFETY_CHECK, target idedata (devicetree + allocation, no compile, about 20 s).
##
## @par Usage
##   python3 tests/l0/safety_allocation/build_check.py --out DIR            # check
##   python3 tests/l0/safety_allocation/build_check.py --out DIR --update-docs  # also rewrite docs/safety/boards
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
BUILD = os.path.join(REPO, '.pio', 'build', 'SAFETY_CHECK')
DOCS = os.path.join(REPO, 'docs', 'safety', 'boards')


def shields():
    out = []
    for p in sorted(glob.glob(os.path.join(REPO, 'zephyr', 'boards', 'shields', '*', '*.overlay'))):
        m = re.match(r'(.+)_v(\d+_\d+_\d+)$', os.path.basename(p)[:-len('.overlay')])
        if m:
            out.append((m.group(1), m.group(2)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--update-docs', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(DOCS, exist_ok=True)
    summary = []
    for name, ver in shields():
        tag = f'{name}_v{ver}'
        env = dict(os.environ, SAFETY_SHIELD=name, SAFETY_SHIELD_VERSION=ver, SAFETY_CHECK_ARGS='')
        shutil.rmtree(BUILD, ignore_errors=True)
        r = subprocess.run([PIO, 'run', '-e', 'SAFETY_CHECK', '-t', 'idedata'], cwd=REPO, env=env,
                           capture_output=True, text=True)
        subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
        log = os.path.join(a.out, f'{tag}.log')
        open(log, 'w').write(r.stdout + r.stderr)
        man = os.path.join(BUILD, 'zephyr', 'safety', 'protection-manifest.json')
        table = os.path.join(BUILD, 'zephyr', 'safety', 'protection-table.md')
        ok = r.returncode == 0 and os.path.exists(man)
        errs = [l for l in (r.stdout + r.stderr).splitlines() if 'ERROR: ' in l or 'devicetree error' in l]
        print(f'@@\tL0-ALC-11.{tag}\t{"PASS" if ok else "FAIL"}\t{tag}: devicetree and protection allocation '
              f'build\tbuilds, every limit covered or excepted\t'
              f'{"builds" if ok else (errs[0].strip()[:160] if errs else f"exit {r.returncode}, see {tag}.log")}',
              flush=True)
        entry = {'shield': tag, 'builds': ok}
        if ok:
            shutil.copy(man, os.path.join(a.out, f'{tag}.json'))
            shutil.copy(table, os.path.join(a.out, f'{tag}.md'))
            doc = os.path.join(DOCS, f'{tag}.md')
            if a.update_docs:
                shutil.copy(table, doc)
            same = os.path.exists(doc) and open(doc).read() == open(table).read()
            print(f'@@\tL0-ALC-12.{tag}\t{"PASS" if same else "FAIL"}\t{tag}: committed protection table matches '
                  f'the build\tdocs/safety/boards/{tag}.md identical\t'
                  f'{"identical" if same else ("missing" if not os.path.exists(doc) else "differs: run build_check.py --update-docs")}',
                  flush=True)
            entry['docs_match'] = same
        summary.append(entry)
    with open(os.path.join(a.out, 'shields.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
