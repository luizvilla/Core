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


def is_tim02(name):
    """@brief Rows of L0-TIM-02: the whole-tick sections and the event rows."""
    sec = name.split('/', 1)[1]
    return sec.startswith('tick') or sec.startswith('ev:')


def event_label(sec):
    """@brief 'ev:vsi 8>2' -> 'vsi PWM_SETTLE → GRID_CONNECT'."""
    t = sec[3:]
    if t.startswith('vsi '):
        a, b = t[4:].split('>')
        return f'vsi {VSI[int(a)]} → {VSI[int(b)]}'
    return t


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


def tool_version(cmd):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        return (r.stdout + r.stderr).strip().splitlines()[0]
    except (OSError, IndexError, subprocess.SubprocessError):
        return 'unknown'


def ini_value(repo, key):
    """@brief First `key = value` in platformio.ini (board and shield as configured)."""
    for path in ('src/app.ini', 'platformio.ini'):
        for ln in open(os.path.join(repo, path)):
            if ln.split('=')[0].strip() == key:
                return ln.split('=', 1)[1].strip()
    return '?'


def manifest(repo, rep, base):
    """@brief Test-case manifest read by tests/common/review_report.py (on-target test cases).
    @return dict: title, purpose, criteria prefixes, conditions and target rows (label, text)."""
    pkg = os.path.expanduser(os.environ.get('PLATFORMIO_PACKAGES_DIR', '~/.platformio/packages'))
    lib = os.path.join(repo, 'owntech', 'lib', 'TIMING', 'single_phase_inverter')
    probe = 'unknown'
    try:
        usb = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
        probe = next((ln.split(' ', 6)[-1] for ln in usb.splitlines() if 'ST-LINK' in ln.upper() or 'STLINK' in ln.upper()), 'not seen by lsusb')
    except OSError:
        pass
    tgt = rep.get('target', {})
    phases = ', '.join(f"{b['name']} {b['ticks'] / 10000:g} s" for b in rep['buckets'])
    acr = tgt.get('flash_acr')
    return {
        'test': 'L0-TIM-01',
        'title': 'L0-TIM-01 · Cycle cost of each controlStep section on the Spin',
        'purpose': 'Locate where the 100 µs critical task spends its time, and catch a change that makes any section '
                   'slower than the committed baseline before it reaches the bench (ADR 0005).',
        'criteria': ['L0-TIM-01'],
        'conditions': [
            ['What runs', 'controlStep() on the production ControlState after controlInit(), then the critical task\'s '
                          'scope snapshot and acquire; env TIMING runs this instead of the tasks (no PWM, no relay, no sensors)'],
            ['Inputs (synthetic)', 'grid 230 V rms / 50 Hz sine, VAC sample every tick; DC bus 400 V; input 36 V, 2 A; '
                                   'AC current 50 mA in phase; relay feedback 200 ticks after the command; bridge PWM '
                                   'acknowledged one tick after the request (as the critical task)'],
            ['Stage sequence', phases + f" ({rep['total_ticks']} ticks in all; repeated stages share a bucket)"],
            ['Boost', 'IDLE, BUSREG and MPPT in OFF; BUSREG in every grid stage'],
            ['Measurement', f"DWT cycle counter at {rep['core_hz'] / 1e6:.0f} MHz; each tick with interrupts locked, ticks "
                            'back to back: a row is the code\'s own cost, without ISRs, sensor reads or PWM writes'],
            ['Objectives', 'a: the scenario reaches PWM_SETTLE, GRID_CONNECTED and RUN without a fault; b: no section '
                           f"exceeds the baseline by more than {int(100 * TOLERANCE)} % + {SLACK_CYCLES} cycles"
                           + (f" (baseline {base.get('commit', '?')}, {base.get('date', '?')})" if base else ' (no baseline yet: INFO)')
                           + f'; c: INFO, controlStep + scope against the {BUDGET_US:.0f} µs budget'],
        ],
        'target': [
            ['Board', f"Spin {ini_value(repo, 'board_version')}, shield {ini_value(repo, 'board_shield')} "
                      f"{ini_value(repo, 'board_shield_version')} (not powered or switched)"],
            ['MCU', f"STM32G4 device 0x{tgt.get('device_id', 0):03x}, revision 0x{tgt.get('revision_id', 0):04x}, "
                    f"{rep['core_hz'] / 1e6:.0f} MHz"],
            ['Flash configuration', (f"FLASH_ACR 0x{acr:08x}: {tgt['flash_wait_states']} wait states, prefetch "
                                     f"{'on' if tgt['flash_prefetch'] else 'off'}, instruction cache "
                                     f"{'on' if tgt['flash_icache'] else 'off'}, data cache {'on' if tgt['flash_dcache'] else 'off'}")
                                    if acr is not None else 'not read'],
            ['Firmware', f"Core {git_commit(repo)}, env TIMING (-DTIMING_HARNESS), inverter library "
                         f"{git_commit(lib) if os.path.isdir(lib) else '?'}"],
            ['Probe and tools', f"{probe}; {tool_version([os.path.join(pkg, 'tool-openocd', 'bin', 'openocd'), '--version'])}; "
                                f"{tool_version([os.path.join(pkg, 'toolchain-gccarmnoneeabi', 'bin', 'arm-none-eabi-gcc'), '--version'])}"],
        ],
        'report': 'timing_report.json',
    }


def tim02(rep, rows, buckets, hz, base):
    """@brief L0-TIM-02 verdicts. @return number of failed criteria."""
    fails = 0
    if not any(b.get('tim02_ticks') for b in rep['buckets']):
        line('L0-TIM-02', 'BLOCKED', 'Whole critical tick per stage and per event', 'on target',
             'no L0-TIM-02 ticks in the report (older TIMING firmware?)')
        return 0
    problems = []
    for b in rep['buckets']:
        if b['tim02_ticks'] != b['ticks']:
            problems.append(f"{b['name']} ran {b['tim02_ticks']} of {b['ticks']} ticks")
        if FAULT in b['tim02_vsi_seen']:
            problems.append(f"{b['name']} faulted")
        for m in REQUIRED.get(b['name'], []):
            if m not in b['tim02_vsi_seen']:
                problems.append(f"{b['name']} never reached {VSI[m]}")
    line('L0-TIM-02.a', 'FAIL' if problems else 'PASS',
         'Scenario ran in the critical task to its end and reached PWM_SETTLE, GRID_CONNECTED and RUN without a fault',
         'all ticks, all reached, no FAULT', '; '.join(problems) if problems else
         f"{sum(b['tim02_ticks'] for b in rep['buckets'])} ticks at 10 kHz")
    fails += bool(problems)
    limit = BUDGET_US * hz / 1e6
    for b in buckets:
        t = rows[f'{b}/tick']
        evs = {n.split('/', 1)[1]: r for n, r in rows.items() if n.startswith(b + '/ev:')}
        at = [event_label(sec) for sec, r in evs.items() if r['max_tick'] == t['max_tick']]
        ok = t['max_cycles'] <= limit
        fails += not ok
        line(f'L0-TIM-02.b-{b.lower()}', 'PASS' if ok else 'FAIL', f'{b}: whole tick worst case within the budget',
             f'≤ {BUDGET_US:.0f} µs', f"worst {us(t['max_cycles'], hz):.1f} µs (mean {us(t['sum_cycles'] / max(t['calls'], 1), hz):.1f}), "
             f"on {', '.join(at) if at else 'a tick with no event'}")
        over = [f'{event_label(sec)} {us(r["max_cycles"], hz):.1f} µs' for sec, r in evs.items() if r['max_cycles'] > limit]
        top = sorted(evs.items(), key=lambda kv: -kv[1]['max_cycles'])[:4]
        ok = not over
        fails += not ok
        line(f'L0-TIM-02.c-{b.lower()}', 'PASS' if ok else 'FAIL', f'{b}: every event tick within the budget',
             f'≤ {BUDGET_US:.0f} µs', ('over: ' + '; '.join(over)) if over else
             ('; '.join(f'{event_label(sec)} {us(r["max_cycles"], hz):.1f} µs' for sec, r in top) or 'no event'))
    if base:
        over, new = [], []
        for n, r in rows.items():
            if not is_tim02(n):
                continue
            ref = base['rows'].get(n)
            if ref is None:
                new.append(n)
            elif r['max_cycles'] > ref * (1.0 + base.get('tolerance', TOLERANCE)) + SLACK_CYCLES:
                over.append(f"{n} {us(r['max_cycles'], hz):.1f} µs > {us(ref * (1 + base.get('tolerance', TOLERANCE)) + SLACK_CYCLES, hz):.1f} µs")
        known = [n for n in rows if is_tim02(n) and n in base['rows']]
        if not known:
            line('L0-TIM-02.d', 'INFO', 'No tick part or event slower than the baseline (no baseline yet)', '-',
                 f'{len(new)} rows to baseline')
        else:
            fails += bool(over)
            line('L0-TIM-02.d', 'FAIL' if over else 'PASS', 'No tick part or event slower than the baseline',
                 f"baseline {base.get('commit', '?')} +{int(100 * base.get('tolerance', TOLERANCE))} %",
                 '; '.join(over) if over else 'all within' + (f'; {len(new)} rows not in baseline' if new else ''))
    return fails


def manifest2(repo, rep, base):
    """@brief L0-TIM-02 test-case manifest (review page, figures 'tim02')."""
    m = manifest(repo, rep, base)
    phases = ', '.join(f"{b['name']} {b['tim02_ticks'] / 10000:g} s" for b in rep['buckets'])
    m.update({
        'test': 'L0-TIM-02', 'figures': 'tim02',
        'title': 'L0-TIM-02 · Whole critical tick per stage and per event, in the critical task',
        'purpose': 'Check that a whole critical tick (sensor reads, controlStep, PWM and relay writes, scope sampling) '
                   'fits in the 80 µs budget in every stage and on every event tick, without the disturbance of the '
                   'running firmware (that is L0-TIM-03).',
        'criteria': ['L0-TIM-02'],
        'conditions': [
            ['What runs', 'the production critical tick (main.cpp criticalTickBody, also called by loop_critical_task) '
                          'from the critical task itself, at 10 kHz on the HRTIM interrupt; ADC acquisition and PWM '
                          'running on the bare Spin (no shield); application and communication tasks not started, no console traffic'],
            ['Inputs (synthetic)', 'the sensors are read, then replaced by the L0-TIM-01 signals (230 V rms / 50 Hz, '
                                   '400 V bus, 36 V / 2 A input, 50 mA AC); relay feedback and bridge acknowledgement '
                                   'from the critical task\'s own models; scope re-armed at each stage start as a bench key does'],
            ['Stage sequence', phases + ' (the L0-TIM-01 sequence, continued in time)'],
            ['Events', 'stage entry, boost mode change, inverter mode change (vsi a → b), relay command, relay feedback, '
                       'bridge enable: each event tick is recorded in its own row'],
            ['Measurement', f"DWT cycle counter at {rep['core_hz'] / 1e6:.0f} MHz around the tick body; nothing preempts "
                            'the critical task (highest priority), so a row is the tick\'s own cost in the real context'],
            ['Objectives', f'a: the scenario ran to its end and reached PWM_SETTLE, GRID_CONNECTED and RUN without a fault; '
                           f'b: the whole tick ≤ {BUDGET_US:.0f} µs in every stage; c: every event tick ≤ {BUDGET_US:.0f} µs; '
                           f'd: no tick part or event slower than the baseline + {int(100 * TOLERANCE)} %'],
        ],
    })
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('report')
    ap.add_argument('--baseline', help='committed baseline to compare with')
    ap.add_argument('--update-baseline', metavar='PATH', help='write this run as the new baseline')
    ap.add_argument('--out', help='L0-TIM-01 test-case folder (timing_tim01.md, target_manifest.json, report)')
    ap.add_argument('--out2', help='L0-TIM-02 test-case folder (timing_tim02.md, target_manifest.json, report)')
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
        sec = [r for n, r in rows.items() if n.startswith(b + '/') and not is_tim02(n)]
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

    fails += tim02(rep, rows, buckets, hz, base)

    if a.out2:
        os.makedirs(a.out2, exist_ok=True)
        with open(os.path.join(a.out2, 'timing_report.json'), 'w') as f:
            json.dump(rep, f, indent=1)
        with open(os.path.join(a.out2, 'target_manifest.json'), 'w') as f:
            json.dump(manifest2(repo, rep, base), f, indent=1, ensure_ascii=False)
        md = ['# L0-TIM-02: whole critical tick per stage and per event', '',
              f"Core {hz / 1e6:.0f} MHz, commit {git_commit(repo)}. Worst case (mean) in µs, budget {BUDGET_US:.0f} µs.", '',
              '| Bucket | tick | sensors | control | apply | scope |', '|---|---|---|---|---|---|']
        for b in buckets:
            cells = [f"{us(rows[f'{b}/{k}']['max_cycles'], hz):.1f} ({us(rows[f'{b}/{k}']['sum_cycles'] / max(rows[f'{b}/{k}']['calls'], 1), hz):.1f})"
                     for k in ('tick', 'tick_sensors', 'tick_control', 'tick_apply', 'tick_scope')]
            md.append(f'| {b} | ' + ' | '.join(cells) + ' |')
        md += ['', '| Bucket | Event | Count | Worst tick (µs) |', '|---|---|---|---|']
        for n, r in rows.items():
            if '/ev:' in n:
                b, sec = n.split('/', 1)
                md.append(f"| {b} | {event_label(sec)} | {r['calls']} | {us(r['max_cycles'], hz):.1f} |")
        with open(os.path.join(a.out2, 'timing_tim02.md'), 'w') as f:
            f.write('\n'.join(md) + '\n')

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, 'timing_report.json'), 'w') as f:
            json.dump(rep, f, indent=1)
        with open(os.path.join(a.out, 'target_manifest.json'), 'w') as f:
            json.dump(manifest(repo, rep, base), f, indent=1, ensure_ascii=False)
        md = ['# L0-TIM-01: cycle cost per section', '',
              f"Core {hz / 1e6:.0f} MHz, {rep['total_ticks']} ticks, commit {git_commit(repo)}, "
              f"{datetime.datetime.now().isoformat(timespec='seconds')}. Worst case and mean in µs; "
              'interrupts locked, so no ISR time is included.', '',
              '| Bucket | ' + ' | '.join(n.split('/')[1] for n in rows if n.startswith(buckets[0] + '/') and not is_tim02(n)) + ' |',
              '|---|' + '---|' * sum(1 for n in rows if n.startswith(buckets[0] + '/') and not is_tim02(n))]
        for b in buckets:
            cells = []
            for n, r in rows.items():
                if n.startswith(b + '/') and not is_tim02(n):
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
        data = {'_doc': 'L0-TIM-01/02 baseline: worst case per section, tick part and event, in core cycles. Written by '
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
