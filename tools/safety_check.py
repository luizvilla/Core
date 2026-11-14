#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file safety_check.py
## @brief Design-time protection check (ADR 0009, docs/safety_new_board.md): runs the build's protection
##        allocation on a DRAFT shield devicetree and prints the coverage report, before any PCB exists.
##
## The draft is a shield overlay (sensors with their Spin pins and ADC channels, legs, safety-thresholds, safe
## outputs), complete on its own or on top of an existing shield (--base). Only the devicetree and the allocation
## run (PlatformIO env SAFETY_CHECK, target idedata, about 20 s); nothing is compiled or flashed. The report is
## never a build failure here: errors are listed for the designer to fix. Attach the printed report to the
## schematic review.
##
## @par Usage
##   tools/safety_check.py my_shield_draft.overlay
##   tools/safety_check.py my_changes.overlay --base twist:1_4_2
##   tools/safety_check.py my_shield_draft.overlay --json report.json
import argparse
import json
import os
import shutil
import subprocess
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
BUILD = os.path.join(REPO, '.pio', 'build', 'SAFETY_CHECK')
sys.path.insert(0, os.path.join(REPO, 'zephyr', 'modules', 'owntech_safety_api', 'scripts'))
from safety_allocation import COMP_INP, COMP_FLT, path_text  # noqa: E402

SPIN_COMP_PINS = {37: 'PB1', 44: 'PA7', 52: 'PA3', 29: 'PA0', 25: 'PC1', 30: 'PA1', 31: 'PB0', 1: 'PB11'}
SPIN_FLT_PINS = {48: 'PA15 (FLT2)', 41: 'PB10 (FLT3)', 16: 'PC10 (FLT6)', 9: 'PC7 (FLT5)', 1: 'PB11 (FLT4)',
                 31: 'PB0 (FLT5)'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('draft', help='draft shield overlay')
    ap.add_argument('--base', help='existing shield the draft builds on, as name:version (e.g. twist:1_4_2)')
    ap.add_argument('--json', help='also write the protection manifest here')
    a = ap.parse_args()
    draft = os.path.abspath(a.draft)
    if not os.path.exists(draft):
        sys.exit(f'safety_check: {draft} not found')
    shield, version = (a.base.split(':') + [''])[:2] if a.base else ('', '')
    env = dict(os.environ, SAFETY_SHIELD=shield, SAFETY_SHIELD_VERSION=version,
               SAFETY_CHECK_ARGS=f'-DSAFETY_CHECK_REPORT_ONLY=1 -DSAFETY_DRAFT_OVERLAY={draft}')
    shutil.rmtree(BUILD, ignore_errors=True)   # PlatformIO does not reconfigure on overlay edits
    r = subprocess.run([PIO, 'run', '-e', 'SAFETY_CHECK', '-t', 'idedata'], cwd=REPO, env=env,
                       capture_output=True, text=True)
    subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
    man_path = os.path.join(BUILD, 'zephyr', 'safety', 'protection-manifest.json')
    if not os.path.exists(man_path):
        print(r.stdout[-4000:] + r.stderr[-4000:])
        sys.exit('safety_check: the devicetree did not build (errors above): fix the draft overlay first')
    man = json.load(open(man_path))
    print(f'Protection check of {os.path.basename(draft)}' + (f' on {a.base}' if a.base else '') +
          f' ({man["shield"]}), ADR 0009\n')
    print(f'{"Limit":12s} {"Low bound":26s} {"High bound":26s} Note')
    for l in man['limits']:
        print(f'{l["sensor"]:12s} {path_text(l["paths"]["low"]):26s} {path_text(l["paths"]["high"]):26s} '
              f'{l.get("exception") or ""}')
    res = man['resources']
    print(f'\nFault inputs: {", ".join(f"{k} {v}" for k, v in res["flt"].items()) or "none used"}')
    print(f'Free fault inputs: {", ".join(res["free_flt"]) or "none"}')
    used_pins = {ch['spin_pin'] for s in man['model']['sensors'].values() for ch in s['channels']}
    free_comp = [f'Spin {sp} ({mcu}, COMP{COMP_INP[mcu][0]} -> FLT{COMP_FLT.get(COMP_INP[mcu][0], "-")})'
                 for sp, mcu in sorted(SPIN_COMP_PINS.items())
                 if sp not in used_pins and f'COMP{COMP_INP[mcu][0]}' not in res['comparators']]
    print(f'Free comparator inputs: {", ".join(free_comp) or "none"}')
    print(f'HRTIM fault pins not used by a sensor: {", ".join(f"Spin {k} {v}" for k, v in sorted(SPIN_FLT_PINS.items()) if k not in used_pins)}')
    for w in man['warnings']:
        print('\nWARNING: ' + w)
    for e in man['errors']:
        print('\nERROR: ' + e)
    if a.json:
        shutil.copy(man_path, a.json)
    print(f'\n{len(man["errors"])} error(s), {len(man["warnings"])} warning(s). '
          'Attach this report to the schematic review (docs/safety_new_board.md).')
    return 1 if man['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
