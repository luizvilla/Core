#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-SUP: CPU supervision on the Spin through an ST-Link (ADR 0009, issue #16).
##
## Builds and flashes env SAFETY_SUP (tests/firmware/safety_supervision), then drives each case over SWD: it
## writes the command word g_safety_sup_cmd, waits for the acknowledgement in g_safety_sup_report (riding
## through the resets the fault cases cause), and judges the results. Prints one result line per criterion
## (@@ format, tests/common/make_record.py) and writes the test-case folder (report.json, results.md).
## Needs a bare Spin with MCUboot and an ST-Link; no scope. It REFLASHES the board: afterwards the board holds
## the SAFETY_SUP firmware, so flash env USB again before any bench session.
## Standard library only.
##
## @par Usage
##   python3 tests/l0/safety_supervision/run.py --out DIR            # build, flash, run every case
##   python3 tests/l0/safety_supervision/run.py --out DIR --no-flash # board already flashed
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
ENV = 'SAFETY_SUP'
ELF = os.path.join(REPO, '.pio', 'build', ENV, 'firmware.elf')
PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
PKG = os.path.expanduser(os.environ.get('PLATFORMIO_PACKAGES_DIR', '~/.platformio/packages'))
OPENOCD = os.path.join(PKG, 'tool-openocd', 'bin', 'openocd')
OPENOCD_SCRIPTS = os.path.join(PKG, 'tool-openocd', 'openocd', 'scripts')
NM = os.path.join(PKG, 'toolchain-gccarmnoneeabi', 'bin', 'arm-none-eabi-nm')

# Mirrors tests/firmware/safety_supervision/main.cpp
MAGIC, LAYOUT, N_RESULTS = 0x31505553, 3, 18
HEAD = ('magic', 'layout', 'boot_count', 'ack', 'busy', 'latched_at_boot', 'pending',
        'fault_reason', 'fault_pc', 'fault_thread', 'fault_context', 'fault_stack', 'fault_lr')
REPORT = struct.Struct('<%dI%dI' % (len(HEAD), N_RESULTS))
CASES = [('CLEAR', 0), ('CONFIG', 1), ('NO_FALSE_RESET', 2), ('SLOW_TASK', 3), ('FATAL_STACK', 4),
         ('FATAL_HARDFAULT', 5), ('CONTROL_STUCK', 6), ('IRQ_MASKED', 7), ('LOCKUP', 8), ('SOFT_RESET_LATCH', 9)]
R = dict(CONFIG=0, TIMEOUT_DEFAULT=1, NO_FALSE_RESET=2, SLOW_TASK=3, STACK_FLAGS=4, STACK_REASON=5,
         STACK_OUTPUTS=6, STACK_CYCLES=7, HARD_FLAGS=8, HARD_REASON=9, HARD_OUTPUTS=10, HARD_CYCLES=11,
         STUCK_FLAGS=12, IRQ_FLAGS=13, LOCKUP_FLAGS=14, SOFTRESET_FLAGS=15, OUTPUT_MASK=16, CORE_HZ=17)
FAULT_FATAL, FAULT_WATCHDOG = 1, 2
K_ERR_CPU_EXCEPTION, K_ERR_STACK_CHK_FAIL = 0, 2
K_ERR_ARM_USAGE_UNDEFINED_INSTRUCTION = 36   # Zephyr >= 4.0 reports ARM fault details (arch.h)
WATCHDOG_TIMEOUT_US = 1000      # declared, ADR 0009
HANDLER_REACTION_US = 10.0      # declared, ADR 0009


def symbol(name):
    out = subprocess.run([NM, '-S', ELF], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[3] == name:
            return int(p[0], 16), int(p[1], 16)
    sys.exit(f'run.py: symbol {name} not in {ELF}')


def thread_name(addr):
    """@brief Data symbol containing addr (e.g. z_main_thread), from the ELF."""
    if not addr:
        return '-'
    best = None
    out = subprocess.run([NM, '-S', ELF], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and int(p[0], 16) <= addr < int(p[0], 16) + int(p[1], 16):
            best = f'{p[3]}+{addr - int(p[0], 16)}'
    return best or '?'


def openocd(*cmds, check=True):
    args = [OPENOCD, '-d0', '-s', OPENOCD_SCRIPTS, '-f', 'interface/stlink.cfg', '-f', 'target/stm32g4x.cfg',
            '-c', 'init']
    for c in cmds:
        args += ['-c', c]
    args += ['-c', 'shutdown']
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if check and r.returncode != 0:
        raise RuntimeError(r.stderr[-1500:])
    return r


def read_report(addr):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'r.bin')
        r = openocd(f'dump_image {path} {addr:#x} {REPORT.size}', check=False)
        if r.returncode != 0 or not os.path.exists(path) or os.path.getsize(path) != REPORT.size:
            return None
        v = REPORT.unpack(open(path, 'rb').read())
    rep = dict(zip(HEAD, v[:len(HEAD)]))
    rep['results'] = list(v[len(HEAD):])
    return rep if rep['magic'] == MAGIC else None


def wait(addr, pred, timeout, what):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rep = read_report(addr)
        if rep is not None and pred(rep):
            return rep
        time.sleep(0.5)
    raise TimeoutError(what)


def line(cid, verdict, text, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{text}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def target_snapshot():
    """@brief Board, MCU, firmware and probe of this run (the review page's snapshot part)."""
    out = openocd('mdw 0xE0042000', check=False).stdout + openocd('mdw 0xE0042000', check=False).stderr
    idc = next((int(l.split()[1], 16) for l in out.splitlines() if l.startswith('0xe0042000:')), None)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    return [['Board', 'Spin, bare (no shield, no power stage)'],
            ['MCU', f'STM32G474RE, DBGMCU_IDCODE {idc:#010x} (device {idc & 0xFFF:#x}, revision {idc >> 16:#x})'
             if idc is not None else 'STM32G474RE (IDCODE not read)'],
            ['Firmware', f'env {ENV} (tests/firmware/safety_supervision) at {sha}{"-dirty" if dirty else ""}'],
            ['Probe', 'ST-Link over SWD, PlatformIO OpenOCD; results read from RAM and TAMP backup registers']]


def write_manifest(out, log):
    """@brief Test-case folder manifest rendered by tests/common/review_report.py (kind 'target')."""
    man = {'title': 'L0-SUP · CPU supervision on the Spin',
           'purpose': 'Checks that the converter reaches its safe state without relying on a healthy CPU (ADR 0009): '
                      'the watchdog, the fatal-error handler and the HRTIM system fault are armed with the PWM, every '
                      'provoked CPU failure turns every PWM output off within its declared fault reaction time, and the '
                      'fault stays latched across the reset until cleared.',
           'criteria': ['L0-SUP-'],
           'conditions': [['Board', 'Bare Spin, no shield, no power stage; PWM units A, C, D, E, F at 200 kHz, 50 %'],
                          ['Fault injection', 'from the test firmware: thread stack overflow (MPU guard), undefined '
                                              'instruction, control task that never returns, interrupts masked, '
                                              'lockup (fault with FAULTMASK set), software reset'],
                          ['Declared times', f'watchdog <= {WATCHDOG_TIMEOUT_US} us without a control task, >= 3 periods '
                                             f'with one; fault entry to outputs disabled <= {HANDLER_REACTION_US:g} us'],
                          ['Timing method', 'DWT cycle counter: marked just before the fault, stamped when the outputs '
                                            'are disabled, kept in TAMP backup registers across the reset'],
                          ['Not covered here', 'the pin levels during a lockup (HRTIM system fault) need a scope: '
                                               'L0-SUP-08.c stays BLOCKED until the SPOC bench']],
           'target': target_snapshot(), 'report': 'report.json', 'figures': 'supervision'}
    with open(os.path.join(out, 'target_manifest.json'), 'w') as f:
        json.dump(man, f, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True, help='test-case folder')
    ap.add_argument('--no-flash', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if not a.no_flash:
        print(f'== build and flash env {ENV}', flush=True)
        with open(os.path.join(a.out, 'build.txt'), 'w') as f:
            r = subprocess.run([PIO, 'run', '-e', ENV, '-t', 'upload'], cwd=REPO, stdout=f, stderr=subprocess.STDOUT)
        if r.returncode != 0:
            line('L0-SUP-00', 'FAIL', 'SAFETY_SUP build or flash', 'exit 0', f'exit {r.returncode}, see build.txt')
            return 1
    addr, size = symbol('g_safety_sup_report')
    if size != REPORT.size:
        sys.exit(f'run.py: g_safety_sup_report is {size} bytes, reader expects {REPORT.size}')
    cmd_addr, _ = symbol('g_safety_sup_cmd')

    rep = wait(addr, lambda r: r['busy'] == 0, 30, 'firmware did not start')
    log = {'cases': []}
    for name, code in CASES:
        before = rep
        seq = (before['ack'] & 0xFFFFFF) + 1
        t0 = time.monotonic()
        openocd(f'mww {cmd_addr:#x} {(seq << 8) | code:#x}')
        try:
            rep = wait(addr, lambda r: r['ack'] == seq and r['busy'] == 0, 20, f'{name} not acknowledged')
            status = 'acknowledged'
        except TimeoutError as e:
            rep = read_report(addr) or before
            status = str(e)
        entry = {'case': name, 'seq': seq, 'status': status, 'seconds': round(time.monotonic() - t0, 2),
                 'boots': rep['boot_count'] - before['boot_count'], 'latched_at_boot': rep['latched_at_boot'],
                 'fault': {k: rep[k] for k in HEAD if k.startswith('fault_')}}
        if rep['latched_at_boot']:
            f = entry['fault']
            print(f"   latched {rep['latched_at_boot']}: reason {f['fault_reason']}, pc {f['fault_pc']:#010x}, "
                  f"thread {f['fault_thread']:#010x} ({thread_name(f['fault_thread'])}), "
                  f"IPSR {f['fault_context'] & 0x1FF}{' in ISR' if f['fault_context'] >> 31 else ''}, "
                  f"stack {f['fault_stack'] & ~0xFFF:#010x} size {(f['fault_stack'] & 0xFFF) * 8}, lr {f['fault_lr']:#010x}", flush=True)
        log['cases'].append(entry)
        print(f"== {name}: {status}, {entry['boots']} reset(s), {entry['seconds']} s", flush=True)
    res = rep['results']
    log['report'] = rep
    boots = {c['case']: c['boots'] for c in log['cases']}
    acked = {c['case']: c['status'] == 'acknowledged' for c in log['cases']}
    hz = res[R['CORE_HZ']] or 170_000_000
    mask = res[R['OUTPUT_MASK']] or 0xFF3

    # L0-SUP-01: arming configuration
    cfg = res[R['CONFIG']]
    line('L0-SUP-01.a', judge(cfg & 0xF == 0xF), 'HRTIM system fault sources connected and locked (SYSCFG_CFGR2)',
         'CLL SPL PVDL ECCL = 1111', f'{cfg & 0xF:04b}')
    line('L0-SUP-01.b', judge(cfg >> 4 & 1 == 1), 'PVD enabled before PVDL locks it', 'PVDE = 1', str(cfg >> 4 & 1))
    line('L0-SUP-01.c', judge(cfg >> 5 & 1 == 1), 'Fault state Inactive on every output of timers A, C, D, E, F',
         'FAULT1 = FAULT2 = 10', 'yes' if cfg >> 5 & 1 else 'no')
    pr, rlr, t_def = cfg >> 8 & 7, cfg >> 16 & 0xFFF, res[R['TIMEOUT_DEFAULT']]
    line('L0-SUP-01.d', judge(acked['CONFIG'] and 0 < t_def <= WATCHDOG_TIMEOUT_US),
         'Watchdog timeout without a control task, worst case over the LSI range',
         f'<= {WATCHDOG_TIMEOUT_US} us', f'{t_def} us (PR {pr}, RLR {rlr})')

    # L0-SUP-02 / 03: no false reset, slow task
    nfr = res[R['NO_FALSE_RESET']]
    line('L0-SUP-02.a', judge(nfr & 1 and boots['NO_FALSE_RESET'] == 0),
         'No reset over 2 s with PWM on, watchdog fed by the kernel timer', '0 resets',
         f"{boots['NO_FALSE_RESET']} resets, flag {nfr & 1}")
    line('L0-SUP-02.b', judge(nfr >> 1 & 1 and boots['NO_FALSE_RESET'] == 0),
         'No reset over 2 s with PWM on, watchdog fed by a 50 us control task', '0 resets, > 30000 calls',
         f"{boots['NO_FALSE_RESET']} resets, flag {nfr >> 1 & 1}")
    st = res[R['SLOW_TASK']]
    line('L0-SUP-03.a', judge(st >> 8 >= 6000), 'Watchdog timeout with a 2 ms control task', '>= 6000 us (3 periods)',
         f'{st >> 8} us')
    line('L0-SUP-03.b', judge(st & 1 and boots['SLOW_TASK'] == 0), 'No reset over 1 s with a 2 ms control task',
         '0 resets', f"{boots['SLOW_TASK']} resets")

    # Fatal cases
    def fatal(cid, case, flags_i, reason_i, outputs_i, cycles_i, reason_exp, what):
        f, reason, outs, cyc = res[flags_i], res[reason_i], res[outputs_i], res[cycles_i]
        line(f'{cid}.a', judge(boots[case] >= 1 and f & 0xFF == FAULT_FATAL), f'{what}: fatal error latched across the reset',
             'latched fatal error, >= 1 reset', f"fault {f & 0xFF}, {boots[case]} reset(s)")
        exp = reason_exp if isinstance(reason_exp, tuple) else (reason_exp,)
        line(f'{cid}.b', judge(reason in exp), f'{what}: Zephyr reason recorded', ' or '.join(map(str, exp)), str(reason))
        line(f'{cid}.c', judge(outs & mask == 0), f'{what}: every switching PWM pin low right after the handler disabled them',
             f'0x000 on mask {mask:#05x}', f'{outs & mask:#05x}')
        us = cyc / hz * 1e6
        line(f'{cid}.d', judge(0 < us <= HANDLER_REACTION_US), f'{what}: time from the provoked fault to the outputs disabled',
             f'<= {HANDLER_REACTION_US} us', f'{us:.2f} us ({cyc} cycles)')
        line(f'{cid}.e', judge(f >> 8 & 1 and f >> 9 & 1 and f >> 10 & 1 and f >> 11 & 1),
             f'{what}: PWM refused while latched; clear refused with PWM on, accepted with PWM off; PWM back after clear',
             'refused, refused, accepted, back', f'flags {f >> 8 & 0xF:04b} (b8 refused, b9 cleared, b10 back, b11 clear refused)')

    fatal('L0-SUP-04', 'FATAL_STACK', R['STACK_FLAGS'], R['STACK_REASON'], R['STACK_OUTPUTS'], R['STACK_CYCLES'],
          K_ERR_STACK_CHK_FAIL, 'Thread stack overflow')
    fatal('L0-SUP-05', 'FATAL_HARDFAULT', R['HARD_FLAGS'], R['HARD_REASON'], R['HARD_OUTPUTS'], R['HARD_CYCLES'],
          (K_ERR_CPU_EXCEPTION, K_ERR_ARM_USAGE_UNDEFINED_INSTRUCTION), 'Undefined instruction')

    def watchdog(cid, case, flags_i, what):
        f = res[flags_i]
        line(f'{cid}.a', judge(boots[case] >= 1 and f & 0xFF == FAULT_WATCHDOG), f'{what}: watchdog reset, fault latched',
             'latched watchdog, >= 1 reset', f"fault {f & 0xFF}, {boots[case]} reset(s)")
        line(f'{cid}.b', judge(f >> 8 & 1 and f >> 9 & 1 and f >> 10 & 1), f'{what}: PWM refused while latched, back after clear',
             'refused, accepted, back', f'flags {f >> 8 & 0x7:03b}')

    watchdog('L0-SUP-06', 'CONTROL_STUCK', R['STUCK_FLAGS'], 'Control task never returns')
    watchdog('L0-SUP-07', 'IRQ_MASKED', R['IRQ_FLAGS'], 'Interrupts masked forever')
    watchdog('L0-SUP-08', 'LOCKUP', R['LOCKUP_FLAGS'], 'Cortex-M4 lockup')
    line('L0-SUP-08.c', 'BLOCKED', 'Cortex-M4 lockup: HRTIM system fault turns the outputs off before the watchdog reset',
         'every PWM pin low within 1 us of the lockup', 'needs a scope on the PWM pins (SPOC bench, #23/#26)')
    sr = res[R['SOFTRESET_FLAGS']]
    line('L0-SUP-09.a', judge(sr & 0xFF == FAULT_FATAL and sr >> 8 & 1), 'A software reset keeps the latched fault',
         'latched before and after the software reset', f'fault {sr & 0xFF}, still latched {sr >> 8 & 1}')
    line('L0-SUP-09.b', judge(sr >> 9 & 1), 'The latched fault clears with clearFault() after a software reset',
         'cleared', str(sr >> 9 & 1))

    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump(log, f, indent=1)
    write_manifest(a.out, log)
    print('The board now runs the SAFETY_SUP test firmware: flash env USB before any bench session.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
