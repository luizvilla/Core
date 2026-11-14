#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-TOV: Safety API timing overhead on a bare Spin, before and after (issue luizvilla/Core#27).
##
## Flashes four builds of tests/firmware/safety_timing one after the other and runs every scenario, each in a fresh
## boot:
##   B0  Core main at 8111a0dc (before the safety work), built by build_before.sh;
##   B1  safety-hw production (env SAFETY_TIME);
##   B1p B1 with the timing probes (env SAFETY_TIME_PROBE): per-block costs of the control interrupt;
##   B2  safety-hw development, run-time self-tests off, disarm() (env SAFETY_TIME_DEV).
## Scenarios: S0 (no safety call) and S1 (old software check on every Twist sensor), at 50 and 100 us; plus a
## software trip under load (B1, B2). Writes timing_tov.json and timing_tov.md (side by side) and the test-case folder.
## It REFLASHES the board: flash env USB again before any bench session.
##
## @par Usage
##   python3 tests/l0/safety_timing/run.py --out DIR
import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'common'))
import target_harness as th  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = (['magic', 'layout', 'boot_count', 'ack', 'busy', 'latched_at_boot', 'pending', 'build', 'scenario',
           'period_us', 'load_us', 'core_hz', 'periods', 'expected', 'gap_min', 'gap_max', 'gap_late',
           'hist_under', 'hist_over'] + [f'h{i}' for i in range(64)]
          + ['bg_lo', 'bg_hi', 'win_lo', 'win_hi', 'bg_longest', 'arm_cycles', 'fault_after', 'react_cycles',
             'react_fault', 'react_reason', 'outputs_on_end'])
MAGIC = 0x31564F54
SCEN = [('S0_50', 1), ('S1_50', 2), ('S0_100', 3), ('S1_100', 4)]
BUILDS = [('B0', 'TOV_BEFORE', SCEN), ('B1', 'SAFETY_TIME', SCEN + [('REACT', 5)]),
          ('B1p', 'SAFETY_TIME_PROBE', SCEN), ('B2', 'SAFETY_TIME_DEV', SCEN + [('REACT', 5)])]
BLOCKS = ['software check', 'dispatch', 'user task', 'read-back', 'plausibility', 'feed']
SAFETY_BLOCKS = [0, 3, 4, 5]
# Provisional limits (docs/safety/timing_overhead.md, "measure first"), to be confirmed with the numbers.
OVERHEAD_SHARE = 0.05      # safety code per period, share of the period
IRQ_OFF_US = 2.0           # longest interrupts-off window caused by the Safety API
JITTER_EXTRA_US = 2.0      # worst task-start deviation, B1 beyond B0
LOAD_EXTRA = 0.05          # CPU taken from the main thread, B1 beyond B0 (share of the CPU)
REACT_PERIODS = 7          # software trip: 5 periods declared, +1 in progress, +1 margin


def flash(name, env, out):
    log = os.path.join(out, f'build_{env}.txt')
    print(f'== build and flash {name} (env {env})', flush=True)
    if env == 'TOV_BEFORE':
        with open(log, 'w') as f:
            return subprocess.run(['bash', os.path.join(HERE, 'build_before.sh'), '--flash'], stdout=f,
                                  stderr=subprocess.STDOUT).returncode
    return th.flash(env, log)


def probes(env):
    a, _ = th.symbol(env, 'g_safety_timing')
    w = th.read_words(a, 22) or [0] * 22
    per = w[1] or 1
    return {'periods': w[1], 'max_us': [w[2 + i] / 170.0 for i in range(6)],
            'mean_us': [(w[8 + 2 * i] | (w[9 + 2 * i] << 32)) / per / 170.0 for i in range(6)],
            'gate_lock_us': w[20] / 170.0}


def selftest(env):
    try:
        a, _ = th.symbol(env, 'g_safety_selftest')
    except SystemExit:
        return None
    s = th.read_words(a, 20) or [0] * 20
    return {'irqoff_us': s[18] / 170.0, 'ram_pass_ms': s[19], 'ram_passes': s[13], 'max_slice_us': s[16]}


def summarise(rep):
    hz = rep['core_hz'] or 170_000_000
    per = rep['period_us']
    us = 1e6 / hz
    bg = rep['bg_lo'] | (rep['bg_hi'] << 32)
    win = (rep['win_lo'] | (rep['win_hi'] << 32)) or 1
    return {'period_us': per, 'periods': rep['periods'], 'expected': rep['expected'],
            'gap_min_us': rep['gap_min'] * us, 'gap_max_us': rep['gap_max'] * us,
            'jitter_us': max(rep['gap_max'] * us - per, per - rep['gap_min'] * us) if rep['periods'] else None,
            'late': rep['gap_late'], 'hist': [rep[f'h{i}'] for i in range(64)],
            'hist_under': rep['hist_under'], 'hist_over': rep['hist_over'],
            'load': 1.0 - bg / win, 'bg_longest_us': rep['bg_longest'] * us, 'arm_us': rep['arm_cycles'] * us,
            'fault_after': rep['fault_after'], 'outputs_on_end': rep['outputs_on_end'],
            'react_us': rep['react_cycles'] * us, 'react_fault': rep['react_fault'],
            'react_reason': rep['react_reason'], 'build': rep['build']}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    J, L = th.judge, th.line
    data = {'cases': [], 'builds': {}}
    acr = None
    for name, env, cases in BUILDS:
        rc = flash(name, env, a.out)
        if rc != 0:
            L('L0-TOV-00', 'FAIL', f'{name} (env {env}) build or flash', 'exit 0', f'exit {rc}, see build_{env}.txt')
            continue
        p = th.Protocol(env, 'g_tov_report', 'g_tov_cmd', FIELDS, MAGIC)
        th.openocd('reset run', check=False)
        res = {}
        for cname, code in cases:
            rep, entry = p.run(f'{name} {cname}', code, timeout=60)
            data['cases'].append(entry)
            if entry['status'] != 'acknowledged' or rep is None:
                res[cname] = {'error': entry['status']}
                continue
            res[cname] = summarise(rep)
            if name == 'B1p':
                res[cname]['probes'] = probes(env)
            st = selftest(env)
            if st:
                res[cname]['selftest'] = st
        if acr is None:
            w = th.read_words(0x40022000, 1)
            acr = w[0] if w else None
        data['builds'][name] = res
    data['flash_acr'] = acr
    with open(os.path.join(a.out, 'timing_tov.json'), 'w') as f:
        json.dump(data, f, indent=1)

    B = data['builds']

    def get(b, c, k):
        return B.get(b, {}).get(c, {}).get(k)

    # L0-TOV-01 safety cost per period (B1p)
    for cname, _ in SCEN:
        pr = get('B1p', cname, 'probes')
        per = 50 if cname.endswith('50') else 100
        if not pr:
            L(f'L0-TOV-01.{cname}', 'FAIL', f'{cname}: safety cost per control period', 'probe record', 'missing')
            continue
        worst = sum(pr['max_us'][i] for i in SAFETY_BLOCKS)
        mean = sum(pr['mean_us'][i] for i in SAFETY_BLOCKS)
        lim = OVERHEAD_SHARE * per
        parts = ', '.join(f'{BLOCKS[i]} {pr["max_us"][i]:.2f}' for i in SAFETY_BLOCKS)
        L(f'L0-TOV-01.{cname}', J(worst <= lim),
          f'{cname}: safety code in the control interrupt per period, worst case (sum of the block maxima; '
          f'provisional limit {OVERHEAD_SHARE:.0%} of the period)', f'<= {lim:.2f} us',
          f'{worst:.2f} us, mean {mean:.2f} us ({parts})')
    # L0-TOV-02 longest interrupts-off window
    st = get('B1', 'S0_50', 'selftest') or {}
    gl = max((get('B1p', c, 'probes') or {}).get('gate_lock_us', 0.0) for c, _ in SCEN)
    off = max(st.get('irqoff_us', 0.0), gl)
    L('L0-TOV-02', J(st and off <= IRQ_OFF_US),
      'Longest window with interrupts off caused by the Safety API: RAM self-test block, HRTIM gate re-check '
      '(provisional limit)', f'<= {IRQ_OFF_US:.2f} us',
      f'{off:.2f} us (RAM block {st.get("irqoff_us", 0):.2f} us, gate {gl:.2f} us)')
    # L0-TOV-03 jitter, L0-TOV-05 load, L0-TOV-04 lost periods
    for cname, _ in SCEN:
        j0, j1, j2 = (get(b, cname, 'jitter_us') for b in ('B0', 'B1', 'B2'))
        if j0 is None or j1 is None:
            L(f'L0-TOV-03.{cname}', 'FAIL', f'{cname}: task-start jitter, after against before', 'both builds ran',
              f'B0 {get("B0", cname, "error")}, B1 {get("B1", cname, "error")}')
        else:
            L(f'L0-TOV-03.{cname}', J(j1 - j0 <= JITTER_EXTRA_US),
              f'{cname}: worst task-start deviation from the period, B1 beyond B0 (B2 for reference)',
              f'<= {JITTER_EXTRA_US:.2f} us more than B0',
              f'B0 {j0:.2f} us, B1 {j1:.2f} us, B2 {j2:.2f} us' if j2 is not None else f'B0 {j0:.2f} us, B1 {j1:.2f} us')
        l0, l1, l2 = (get(b, cname, 'load') for b in ('B0', 'B1', 'B2'))
        if l0 is not None and l1 is not None:
            L(f'L0-TOV-05.{cname}', J(l1 - l0 <= LOAD_EXTRA),
              f'{cname}: CPU taken from the main thread (interrupts and work queue), B1 beyond B0 (B2 for reference)',
              f'<= {LOAD_EXTRA:.0%} more than B0',
              f'B0 {l0:.1%}, B1 {l1:.1%}, B2 {l2:.1%}' if l2 is not None else f'B0 {l0:.1%}, B1 {l1:.1%}')
        bad = []
        for b in ('B0', 'B1', 'B2'):
            r = B.get(b, {}).get(cname, {})
            if 'error' in r or not r:
                bad.append(f'{b} {r.get("error", "missing")}')
            elif r['late'] or r['periods'] + 2 < r['expected'] or r['fault_after'] or not r['outputs_on_end']:
                bad.append(f'{b} late {r["late"]}, {r["periods"]}/{r["expected"]} periods, fault {r["fault_after"]} '
                           f'(detail {r["react_reason"]:#x}), outputs {r["outputs_on_end"]}')
        L(f'L0-TOV-04.{cname}', J(not bad), f'{cname}: no lost period, no fault, PWM still on at the end (B0, B1, B2)',
          'none late, all periods, no fault', '; '.join(bad) or 'all clean')
    # L0-TOV-06 reaction under load
    for b in ('B1', 'B2'):
        r = B.get(b, {}).get('REACT', {})
        lim = REACT_PERIODS * 50
        ok = r and 'error' not in r and r['react_fault'] == 3 and r['react_reason'] == 0x403 and 0 < r['react_us'] <= lim
        L(f'L0-TOV-06.{b}', J(ok), f'{b}: software trip under load (50 us task, 10 us load, dispatch, self-tests): '
          'watch to outputs stopped', f'<= {lim} us, TRIP 0x403',
          f'{r.get("react_us", 0):.0f} us, fault {r.get("react_fault")}, detail {r.get("react_reason", 0):#x}'
          if r and 'error' not in r else str(r.get('error', 'missing')))
    # L0-TOV-07 information
    arm = ', '.join(f'{b} {get(b, "S0_50", "arm_us") or 0:.0f} us' for b in ('B0', 'B1', 'B2'))
    L('L0-TOV-07.a', 'INFO', 'First PWM output enable of 5 units (arming of the protection in B1), 50 us task',
      'information', arm)
    st1 = get('B1', 'S0_100', 'selftest') or {}
    L('L0-TOV-07.b', 'INFO', 'Run-time self-tests (B1): longest slice, RAM passes and pass duration within the window',
      'information', f'slice {st1.get("max_slice_us", 0)} us, {st1.get("ram_passes", 0)} passes, '
      f'last pass {st1.get("ram_pass_ms", 0)} ms')
    lg = ', '.join(f'{b} {get(b, "S0_50", "bg_longest_us") or 0:.0f} us' for b in ('B0', 'B1', 'B2'))
    L('L0-TOV-07.c', 'INFO', 'Longest stretch taken from the main thread (control interrupt plus work queue), 50 us',
      'information', lg)
    pr = get('B1p', 'S0_100', 'probes') or {}
    if pr:
        L('L0-TOV-07.d', 'INFO', 'Rest of the control interrupt (B1p, S0, 100 us): data dispatch, user task',
          'information', f'dispatch {pr["mean_us"][1]:.2f} us mean ({pr["max_us"][1]:.2f} max), user '
          f'{pr["mean_us"][2]:.2f} us')
    if acr is not None:
        L('L0-TOV-07.e', 'INFO', 'Flash wait states (FLASH_ACR LATENCY) on this firmware', 'information',
          f'{acr & 0xF} (4 expected at 170 MHz)')

    write_table(a.out, data)
    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump({'cases': data['cases']}, f, indent=1)
    th.write_target_manifest(
        a.out, 'L0-TOV · Safety API timing overhead',
        'Measures what the Safety API costs the control task, before (Core main) and after (safety-hw): cost of each '
        'block of the control interrupt, task-start jitter, CPU left to the application, the longest window with '
        'interrupts off, and the software reaction time under load (issue #27, docs/safety/timing_overhead.md).',
        ['L0-TOV-'],
        [['Builds', 'B0 Core main 8111a0dc; B1 safety-hw production; B1p B1 with probes; B2 development, run-time '
                    'self-tests off, disarm(); the same firmware source, Twist 1.4.1 devicetree'],
         ['Scenarios', 'S0 no safety call; S1 old software check on every Twist sensor; control task 50 and 100 us on '
                       'the HRTIM, 10 us busy load, data dispatch, PWM A, C, D, E, F at 200 kHz; 4 s per scenario'],
         ['Measures', 'DWT cycles: gap between task starts, cycles left to a main-thread loop, probes between the '
                      'blocks of the control interrupt (B1p only), interrupts-off window of the RAM self-test block'],
         ['Limits', 'provisional, "measure first": safety per period <= 5 % of the period, interrupts off <= 2 us, '
                    'jitter <= +2 us and CPU <= +5 % against B0'],
         ['Flash', '4 wait states in every build (RM0440 Table 9; Zephyr\'s clock driver sets 8): safety-hw carries '
                   'the perf(spin) fix, B0 gets the same patch (flash_ws4.patch)']],
        'B0 + SAFETY_TIME + SAFETY_TIME_PROBE + SAFETY_TIME_DEV', 'supervision')
    print('The board now runs the SAFETY_TIME_DEV test firmware: flash env USB before any bench session.')
    return 0


def write_table(out, data):
    B = data['builds']
    rows = ['# L0-TOV results, side by side', '',
            '| Scenario | Build | Jitter (µs) | Gap min–max (µs) | CPU taken | Late | Periods | Arming (µs) |',
            '|---|---|---|---|---|---|---|---|']
    for cname, _ in SCEN:
        for b in ('B0', 'B1', 'B2'):
            r = B.get(b, {}).get(cname)
            if not r or 'error' in r:
                rows.append(f'| {cname} | {b} | {r.get("error") if r else "missing"} | | | | | |')
                continue
            rows.append(f'| {cname} | {b} | {r["jitter_us"]:.2f} | {r["gap_min_us"]:.1f}–{r["gap_max_us"]:.1f} | '
                        f'{r["load"]:.1%} | {r["late"]} | {r["periods"]}/{r["expected"]} | {r["arm_us"]:.0f} |')
    rows += ['', '## Control interrupt per block (B1p, mean / max µs)', '',
             '| Scenario | ' + ' | '.join(BLOCKS) + ' | safety total |', '|---|' + '---|' * (len(BLOCKS) + 1)]
    for cname, _ in SCEN:
        pr = B.get('B1p', {}).get(cname, {}).get('probes')
        if not pr:
            continue
        cells = [f'{pr["mean_us"][i]:.2f} / {pr["max_us"][i]:.2f}' for i in range(len(BLOCKS))]
        tot = (sum(pr['mean_us'][i] for i in SAFETY_BLOCKS), sum(pr['max_us'][i] for i in SAFETY_BLOCKS))
        rows.append(f'| {cname} | ' + ' | '.join(cells) + f' | {tot[0]:.2f} / {tot[1]:.2f} |')
    with open(os.path.join(out, 'timing_tov.md'), 'w') as f:
        f.write('\n'.join(rows) + '\n')


if __name__ == '__main__':
    sys.exit(main())
