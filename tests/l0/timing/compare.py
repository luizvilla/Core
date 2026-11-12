#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file compare.py
## @brief Compares the L0-TIM results of two campaigns (before, after) and prints a Markdown table per stage.
##
## Used to track the timing optimizations one change at a time (docs/testing/timing-optimization.md). Means and
## worst cases in µs from each campaign's l0_tim01/timing_report.json; the interrupt time per period is the
## L0-TIM-03 task API work (safety check + ADC data dispatch) plus the task tick, without the test harness.
## Standard library only.
##
## @par Usage
##   python3 tests/l0/timing/compare.py tests/out/<before> tests/out/<after> [--rows 3:tick_apply,3:tick_scope]
import argparse
import json
import os
import sys


def load(campaign):
    path = os.path.join(campaign, 'l0_tim01', 'timing_report.json')
    rep = json.load(open(path))
    return rep, {r['name']: r for r in rep['rows']}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('before')
    ap.add_argument('after')
    ap.add_argument('--rows', default='', help='extra rows to compare (section names after "<bucket>/"), comma-separated')
    a = ap.parse_args()
    (ra, A), (rb, B) = load(a.before), load(a.after)
    hz = ra['core_hz']
    mean = lambda R, k: R[k]['sum_cycles'] / max(R[k]['calls'], 1) * 1e6 / hz if k in R and R[k]['calls'] else float('nan')
    worst = lambda R, k: R[k]['max_cycles'] * 1e6 / hz if k in R and R[k]['calls'] else float('nan')
    irq = lambda R, b: mean(R, f'{b}/3:safety') + mean(R, f'{b}/3:dispatch') + mean(R, f'{b}/3:tick')
    extra = [x for x in a.rows.split(',') if x]
    head = ['Stage', 'Interrupt / period (mean)', 'Task tick worst', 'Gap worst', 'Lost'] + [f'{x} (mean)' for x in extra]
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    tot = [0.0, 0.0]
    for ba, bb in zip(ra['buckets'], rb['buckets']):
        b = ba['name']
        ia, ib = irq(A, b), irq(B, b)
        tot[0] += ia
        tot[1] += ib
        cells = [b, f'{ia:.1f} → {ib:.1f} ({ib - ia:+.1f})',
                 f"{worst(A, b + '/3:tick'):.1f} → {worst(B, b + '/3:tick'):.1f}",
                 f"{worst(A, b + '/3:gap'):.1f} → {worst(B, b + '/3:gap'):.1f}",
                 f"{ba.get('tim03_lost', 0)} → {bb.get('tim03_lost', 0)}"]
        cells += [f"{mean(A, f'{b}/{x}'):.2f} → {mean(B, f'{b}/{x}'):.2f}" for x in extra]
        print('| ' + ' | '.join(cells) + ' |')
    n = len(ra['buckets'])
    print(f'\nMean change of the interrupt time per period over the {n} stages: {(tot[1] - tot[0]) / n:+.2f} µs '
          f'({os.path.basename(a.before.rstrip("/"))} → {os.path.basename(a.after.rstrip("/"))})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
