#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file evaluate.py
## @brief L0-TIM-01 verdicts from a report read by read_report.py; writes the result lines and a table.
##
## Criteria (procedure §6, tab Timing):
##   L0-TIM-01.a            the scenario reached every inverter state it times, without a fault
##   L0-TIM-01.b-<bucket>   no section slower than the committed baseline + tolerance (INFO without a baseline)
##   L0-TIM-01.c-<bucket>   INFO: controlStep + scope worst case against the 80 µs budget of L0-TIM-02/03
## Standard library only.
##
## @par Usage
##   python3 tests/l0/timing/evaluate.py report.json --baseline tests/l0/timing/baseline.json --out DIR
##   python3 tests/l0/timing/evaluate.py report.json --update-baseline tests/l0/timing/baseline.json
import argparse
import datetime
import json
import os
import subprocess
import sys

PERIOD_US = 100.0
BUDGET_US = 80.0          # ADR 0005: 80 % of the control period
TOLERANCE = 0.10          # +10 % on each section's worst case
SLACK_CYCLES = 50         # absolute slack, so a 1 µs section does not fail on a few cycles
VSI = ['OFF', 'VOLTAGE_SYNC', 'GRID_CONNECT', 'GRID_CONNECTED', 'RUN', 'STOPPING', 'DISCONNECT', 'FAULT',
       'PWM_SETTLE', 'PWM_WAIT']
# Bucket -> inverter states it must reach for its rows to mean what their names say.
REQUIRED = {'SETTLE_c': [8], 'CONNECT_g': [3], 'INJECT_n': [4], 'FOLLOW_CONN_G': [3]}
FAULT = 7


def line(cid, verdict, desc, limit, measured):
    """@brief One result line (tests/common/README.md)."""
    print(f'@@\t{cid}\t{verdict}\t{desc}\t{limit}\t{measured}')


def us(cycles, hz):
    return cycles * 1e6 / hz


def git_commit(repo):
    try:
        sha = subprocess.run(['git', '-C', repo, 'rev-parse', '--short=7', 'HEAD'], capture_output=True,
                             text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', repo, 'status', '--porcelain'], capture_output=True, text=True).stdout
        return sha + ('-dirty' if dirty.strip() else '')
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('report')
    ap.add_argument('--baseline', help='committed baseline to compare with')
    ap.add_argument('--update-baseline', metavar='PATH', help='write this run as the new baseline')
    ap.add_argument('--out', help='folder for timing_tim01.md and .json')
    a = ap.parse_args()
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    rep = json.load(open(a.report))
    if rep.get('state') != 'DONE':
        line('L0-TIM-01.a', 'FAIL', 'Timing harness completed on the target', 'state DONE',
             f"state {rep.get('state')}, error {rep.get('error_code')}")
        return 1
    hz = rep['core_hz']
    rows = {r['name']: r for r in rep['rows']}
    buckets = [b['name'] for b in rep['buckets']]

    # .a coverage
    problems = []
    for b in rep['buckets']:
        if FAULT in b['vsi_seen']:
            problems.append(f"{b['name']} faulted")
        for m in REQUIRED.get(b['name'], []):
            if m not in b['vsi_seen']:
                problems.append(f"{b['name']} never reached {VSI[m]}")
    missing = [b for b in REQUIRED if b not in buckets]
    problems += [f'{b} not run' for b in missing]
    line('L0-TIM-01.a', 'FAIL' if problems else 'PASS',
         'Scenario reached every timed inverter state (PWM_SETTLE, GRID_CONNECTED, RUN) without a fault',
         'all reached, no FAULT', '; '.join(problems) if problems else
         f"{rep['total_ticks']} ticks, {len(buckets)} buckets")

    base = None
    if a.baseline and os.path.exists(a.baseline):
        base = json.load(open(a.baseline))
    fails = 0
    for b in buckets:
        sec = [r for n, r in rows.items() if n.startswith(b + '/')]
        cid = f'L0-TIM-01.b-{b.lower()}'
        desc = f'{b}: no section slower than the baseline'
        if base is None:
            worst = max(sec, key=lambda r: r['max_cycles'])
            line(cid, 'INFO', desc + ' (no baseline yet)', '-',
                 f"worst section {worst['name'].split('/')[1]} {us(worst['max_cycles'], hz):.2f} µs")
            continue
        over = []
        for r in sec:
            ref = base['rows'].get(r['name'])
            if ref is None:
                continue
            limit = ref * (1.0 + base.get('tolerance', TOLERANCE)) + SLACK_CYCLES
            if r['max_cycles'] > limit:
                over.append(f"{r['name'].split('/')[1]} {us(r['max_cycles'], hz):.2f} µs > "
                            f"{us(limit, hz):.2f} µs")
        new = [r['name'] for r in sec if r['name'] not in base['rows']]
        verdict = 'FAIL' if over else 'PASS'
        fails += bool(over)
        line(cid, verdict, desc, f"baseline {base.get('commit', '?')} +{int(100 * base.get('tolerance', TOLERANCE))} %",
             '; '.join(over) if over else ('all within' + (f"; not in baseline: {', '.join(new)}" if new else '')))

    for b in buckets:
        cs = rows[f'{b}/controlStep']['max_cycles']
        sc = rows[f'{b}/scope']['max_cycles']
        total = us(cs + sc, hz)
        line(f'L0-TIM-01.c-{b.lower()}', 'INFO', f'{b}: controlStep + scope worst case (no ISR, no sensors, no PWM)',
             f'context: budget {BUDGET_US:.0f} µs (L0-TIM-02/03)', f'{total:.1f} µs = {total / PERIOD_US * 100:.0f} % of the period')

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, 'timing_tim01.json'), 'w') as f:
            json.dump(rep, f, indent=1)
        md = ['# L0-TIM-01: cycle cost per section', '',
              f"Core {hz / 1e6:.0f} MHz, {rep['total_ticks']} ticks, commit {git_commit(repo)}, "
              f"{datetime.datetime.now().isoformat(timespec='seconds')}. Worst case and mean in µs; "
              'interrupts locked, so no ISR time is included.', '',
              '| Bucket | ' + ' | '.join(n.split('/')[1] for n in rows if n.startswith(buckets[0] + '/')) + ' |',
              '|---|' + '---|' * sum(1 for n in rows if n.startswith(buckets[0] + '/'))]
        for b in buckets:
            cells = []
            for n, r in rows.items():
                if n.startswith(b + '/'):
                    mean = r['sum_cycles'] / max(r['calls'], 1)
                    cells.append(f"{us(r['max_cycles'], hz):.1f} ({us(mean, hz):.1f})")
            md.append(f'| {b} | ' + ' | '.join(cells) + ' |')
        md += ['', 'Inverter states seen per bucket:', '']
        md += [f"- {b['name']}: {', '.join(VSI[m] for m in b['vsi_seen'])}" for b in rep['buckets']]
        with open(os.path.join(a.out, 'timing_tim01.md'), 'w') as f:
            f.write('\n'.join(md) + '\n')

    if a.update_baseline:
        if problems:
            print('evaluate: coverage failed, baseline not written', file=sys.stderr)
            return 1
        data = {'_doc': 'L0-TIM-01 baseline: worst case per section, in core cycles. Written by '
                        'tests/l0/timing/run.sh --update-baseline; commit it with the change that justifies it.',
                'commit': git_commit(repo), 'date': datetime.date.today().isoformat(), 'core_hz': hz,
                'tolerance': TOLERANCE, 'rows': {n: r['max_cycles'] for n, r in rows.items()}}
        with open(a.update_baseline, 'w') as f:
            json.dump(data, f, indent=1)
            f.write('\n')
        print(f'evaluate: baseline written to {a.update_baseline}', file=sys.stderr)
    return 1 if (problems or fails) else 0


if __name__ == '__main__':
    sys.exit(main())
