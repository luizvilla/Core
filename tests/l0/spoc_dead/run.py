#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-CPU: the protection with the CPU dead, on the SPOC bench (ADR 0009, issue luizvilla/Core#23, C12;
##        IEC 62109-1 Annex B route (a): each protection must work with the CPU dead).
##
## Part 1, L0-CPU-01 (firmware SAFETY_SUP, bare Spin, no trips): the supervision firmware's lockup case (FAULTMASK
## set, then a fault) with a marker on PA6 a few cycles before. Per PWM pin, the scope captures the marker (SPOC CH1)
## and the pin (CH3) on one armed single trigger; the pin must be low and stay low within 1 µs of the marker (the
## HRTIM system fault, SYSCFG CFGR2 CLL). This is L0-SUP-08.c.
## Part 2, L0-CPU-02 (firmware SAFETY_L0_TWIST): for each hardware path of the manifest that the bench can drive,
## the CUT is armed with the PWM on, then halted over SWD (watchdog frozen in debug, HRTIM running), and the
## threshold is crossed by writing the CUT's DAC register over SWD: no instruction runs. The HRTIM fault flag of
## the path, the output enables and the PWM pin levels are read over SWD before the CPU is resumed.
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_dead/run.py --out DIR
import argparse
import json
import os
import re
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path[:0] = [os.path.join(REPO, 'tests', 'common'), os.path.join(REPO, 'tests', 'common', 'spoc')]
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location('spoc_trip_run', os.path.join(REPO, 'tests', 'l0', 'spoc_trip', 'run.py'))
trip = importlib.util.module_from_spec(_spec)   # tests/l0/spoc_trip/run.py (both runners are run.py)
_spec.loader.exec_module(trip)
import serial  # noqa: E402
import rigol_mho  # noqa: E402
from spoc import boards  # noqa: E402
from spoc.injector import Injector  # noqa: E402
from mux_controller import MuxController, PIN_TO_ROUTE  # noqa: E402

PIO = trip.PIO
PKG = os.path.expanduser(os.environ.get('PLATFORMIO_PACKAGES_DIR', '~/.platformio/packages'))
OPENOCD = os.path.join(PKG, 'tool-openocd', 'bin', 'openocd')
OPENOCD_SCRIPTS = os.path.join(PKG, 'tool-openocd', 'openocd', 'scripts')
NM = os.path.join(PKG, 'toolchain-gccarmnoneeabi', 'bin', 'arm-none-eabi-nm')

HRTIM_ISR, HRTIM_OENR = 0x40016B88, 0x40016B94
DAC2_DHR12R1 = 0x50000C08
IDR = {'A': 0x48000010, 'B': 0x48000410, 'C': 0x48000810}
DBG_APB1FZR1, DBG_APB2FZR = 0xE0042008, 0xE004200C
IWDG_STOP, HRTIM_STOP = 1 << 12, 1 << 26
TIMER_PINS = {'a': ('PA8', 'PA9'), 'b': ('PA10', 'PA11'), 'c': ('PB12', 'PB13'), 'd': ('PB14', 'PB15'),
              'e': ('PC8', 'PC9'), 'f': ('PC6', 'PC7')}
SUP_PINS = TIMER_PINS['a'] + TIMER_PINS['c'] + TIMER_PINS['d'] + TIMER_PINS['e'] + TIMER_PINS['f']
SUP_LOCKUP, SUP_CLEAR = 8, 0
LOCKUP_LIMIT_S = 1e-6
BEYOND_MV = 50.0                 # DAC level past the threshold, written over SWD
TIMEBASE_S = 2e-6                # 20 µs on screen, trigger in the middle
MARKER_TRIGGER_V = 2.5


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def openocd(*cmds, timeout=60):
    args = [OPENOCD, '-d0', '-s', OPENOCD_SCRIPTS, '-f', 'interface/stlink.cfg', '-f', 'target/stm32g4x.cfg',
            '-c', 'init']
    for c in cmds:
        args += ['-c', c]
    args += ['-c', 'shutdown']
    r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return r.stdout + r.stderr


def words(text, tag):
    """Value printed by `echo "<tag> [capture {mdw ADDR}]"`."""
    m = re.search(rf'{tag} 0x[0-9a-fA-F]+: ([0-9a-fA-F]+)', text)
    return int(m.group(1), 16) if m else None


def flash(env, out):
    r = subprocess.run([PIO, 'run', '-e', env, '-t', 'upload'], cwd=REPO, capture_output=True, text=True)
    subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
    with open(os.path.join(out, f'build_{env}.log'), 'w') as fh:
        fh.write(r.stdout[-20000:] + r.stderr[-20000:])
    if r.returncode != 0 or 'Verified OK' not in r.stdout + r.stderr:
        raise RuntimeError(f'build or flash of {env} failed (build_{env}.log)')


def symbol(elf, name):
    out = subprocess.run([NM, '-S', elf], check=True, capture_output=True, text=True).stdout
    for ln in out.splitlines():
        p = ln.split()
        if len(p) == 4 and p[3] == name:
            return int(p[0], 16)
    raise RuntimeError(f'{name} not in {elf}')


def pin_level(idr_values, pin):
    return (idr_values[pin[1]] >> int(pin[2:])) & 1


# ---------------------------------------------------------------- part 1: lockup, per PWM pin

class SupFirmware:
    """The supervision firmware's command protocol over SWD (tests/l0/safety_supervision/run.py)."""

    def __init__(self):
        elf = os.path.join(REPO, '.pio', 'build', 'SAFETY_SUP', 'firmware.elf')
        self.report = symbol(elf, 'g_safety_sup_report')
        self.cmd = symbol(elf, 'g_safety_sup_cmd')

    def head(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'r.bin')
            openocd(f'dump_image {path} {self.report:#x} 32')
            if not os.path.exists(path) or os.path.getsize(path) != 32:
                return None
            v = struct.unpack('<8I', open(path, 'rb').read())
        return dict(magic=v[0], boot_count=v[2], ack=v[3], busy=v[4], latched=v[5])

    def run_case(self, code, timeout=20):
        h = self.wait(lambda r: r['busy'] == 0, 30)
        seq = (h['ack'] & 0xFFFFFF) + 1
        openocd(f'mww {self.cmd:#x} {(seq << 8) | code:#x}')
        return seq

    def wait(self, pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            h = self.head()
            if h and h['magic'] == 0x31505553 and pred(h):
                return h
            time.sleep(0.5)
        raise TimeoutError('supervision firmware did not answer')


def arm_single(scope, channel, level):
    scope.write(':TRIGger:MODE EDGE')
    scope.write(f':TRIGger:EDGE:SOURce CHANnel{channel}')
    scope.write(':TRIGger:EDGE:SLOPe POSitive')
    scope.write(f':TRIGger:EDGE:LEVel {level}')
    scope.write(':TRIGger:SWEep NORMal')
    scope.write(':SINGle')
    time.sleep(0.5)


def wait_stopped(scope, timeout=15):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if scope.query(':TRIGger:STATus?').strip().upper().startswith('STOP'):
            return True
        time.sleep(0.2)
    return False


def crossing(t, v, level, start=0):
    for i in range(max(start, 1), len(v)):
        if v[i - 1] < level <= v[i]:
            return t[i - 1] + (level - v[i - 1]) * (t[i] - t[i - 1]) / (v[i] - v[i - 1])
    return None


def lockup_case(scope, mux, sup, pin):
    mux.disable_all()
    sh = mux._require_shell()
    for ch, p in (('ch1', 'PA6'), ('ch3', pin)):
        r = PIN_TO_ROUTE[p]
        mux._set_mux_group(sh, mux._mux_path(r['mux'], ch), r['input_index'], True)
    arm_single(scope, 1, MARKER_TRIGGER_V)   # above the PWM crosstalk on CH1 (up to ~1.9 V at 1.65 V trigger)
    seq = sup.run_case(SUP_LOCKUP)
    caught = wait_stopped(scope)
    res = {'pin': pin, 'seq': seq, 'captured': caught}
    if caught:
        tm, vm = scope.waveform(1)
        tp, vp = scope.waveform(3)
        t0 = crossing(tm, vm, MARKER_TRIGGER_V)
        marker_high = [x for t, x in zip(tm, vm) if t0 is not None and t0 + 0.5e-6 <= t <= t0 + 5e-6]
        res['marker_ok'] = bool(marker_high) and min(marker_high) > 2.8
        i0 = next((i for i, t in enumerate(tp) if t0 is not None and t >= t0), len(tp))
        before = [x for t, x in zip(tp, vp) if t0 is not None and t0 - 10e-6 <= t < t0]
        switching = bool(before) and max(before) > 2.5 and min(before) < 0.8
        last_high = max((tp[i] for i in range(i0, len(tp)) if vp[i] > 1.65), default=None)
        off = 0.0 if last_high is None else last_high - t0
        res.update(t_marker=t0, switching_before=switching, off_s=off,
                   wave={'t': tp[::4], 'pin': vp[::4], 'marker': vm[::4]})
    sup.wait(lambda r: r['ack'] == seq and r['busy'] == 0, 30)   # watchdog reset, evaluation, clear
    return res


def part1(out, scope, mux):
    flash('SAFETY_SUP', out)
    time.sleep(3)
    sup = SupFirmware()
    sup.wait(lambda r: r['busy'] == 0, 30)
    seq = sup.run_case(SUP_CLEAR)
    sup.wait(lambda r: r['ack'] == seq and r['busy'] == 0, 30)   # else the next command could reuse its sequence
    scope.set_timebase(TIMEBASE_S)
    results = []
    for k, pin in enumerate(SUP_PINS):
        r = lockup_case(scope, mux, sup, pin)
        if not r['captured']:
            r = dict(lockup_case(scope, mux, sup, pin), retried=True)   # one retry for a missed trigger
        results.append(r)
        cid = f'L0-CPU-01.{pin}'
        what = f'Cortex-M4 lockup: {pin} (HRTIM output) after the marker'
        if not r['captured']:
            line(cid, 'FAIL', what, 'low within 1 µs', 'no trigger on the marker (PA6 on SPOC CH1)')
            continue
        ok = r['marker_ok'] and r['switching_before'] and r['off_s'] <= LOCKUP_LIMIT_S
        line(cid, judge(ok), what, f'marker high, pin switching before, low and staying low within '
             f'{LOCKUP_LIMIT_S * 1e6:g} µs',
             f"marker {'high' if r['marker_ok'] else 'NOT high (no real marker edge)'}; "
             f"{'switching' if r['switching_before'] else 'NOT switching'} before; last high "
             f"{r['off_s'] * 1e9:.0f} ns after the marker")
    mux.disable_all()
    return results


# ---------------------------------------------------------------- part 2: trips with the CPU halted

def halted_trip(b, c):
    """Arms, halts the CUT, crosses the threshold by a DAC register write over SWD, reads the result, resumes."""
    high = c['bound'] == 'high'
    thr = c['threshold_mV']
    start = min(max(thr - (1 if high else -1) * trip.START_MV, 20.0), b.vref - 20.0)
    beyond = min(max(thr + (1 if high else -1) * BEYOND_MV, 0.0), b.vref)
    code = int(round(beyond * 4095 / b.vref))
    res = dict(c, start_mV=start, beyond_mV=beyond, routes=b.route(c['pin'], c['holds']))
    ok, why, _ = b.arm(start, None)
    if not ok:
        res['error'] = why
        return res
    legs = [TIMER_PINS[leg['timer'][-1]] for leg in b.man['legs']]
    pins = [p for pair in legs for p in pair]
    cmds = ['halt', f'mmw {DBG_APB1FZR1:#x} {IWDG_STOP:#x} 0', f'mmw {DBG_APB2FZR:#x} 0 {HRTIM_STOP:#x}',
            f'echo "PC0 [capture {{reg pc}}]"',
            f'echo "OENR0 [capture {{mdw {HRTIM_OENR:#x}}}]"', f'echo "ISR0 [capture {{mdw {HRTIM_ISR:#x}}}]"',
            f'mww {DAC2_DHR12R1:#x} {code:#x}', 'sleep 20',
            f'echo "PC1 [capture {{reg pc}}]"',
            f'echo "OENR1 [capture {{mdw {HRTIM_OENR:#x}}}]"', f'echo "ISR1 [capture {{mdw {HRTIM_ISR:#x}}}]"']
    cmds += [f'echo "IDR{p} [capture {{mdw {IDR[p]:#x}}}]"' for p in sorted({x[1] for x in pins})]
    cmds += ['resume']
    text = openocd(*cmds)
    pcs = re.findall(r'PC[01] pc \(/32\): (0x[0-9a-fA-F]+)', text)
    idr = {p: words(text, f'IDR{p}') for p in sorted({x[1] for x in pins})}
    flt = c['path']['flt']
    bit = 1 << (6 if flt == 6 else flt - 1)
    res.update(oenr_before=words(text, 'OENR0'), isr_before=words(text, 'ISR0'), oenr_after=words(text, 'OENR1'),
               isr_after=words(text, 'ISR1'), pcs=pcs, flt_bit=bit,
               pins={p: (None if idr[p[1]] is None else pin_level(idr, p)) for p in pins}, dac_code=code)
    time.sleep(0.3)
    st = b.safety()
    res['state_after_resume'] = st
    b.pwm(False)
    return res


def part2(out, man_env='SAFETY_L0_TWIST', shield='twist:1_4_2'):
    trip.production_manifest(shield, out)          # keeps the build order of spoc_trip (PlatformIO wipes builds)
    man = trip.build_and_flash(man_env, out)
    trip.reset_cut()
    time.sleep(5)
    b = trip.Bench(man)
    results = []
    try:
        b.vref = float(b.group('Inj')['rVref_mV'])
        wired, _ = b.wire_tp5_tp7()
        held = trip.held_sensors(man)
        armed, why = b.first_arming(held)
        if not armed:
            raise RuntimeError(f'first arming refused: {why}')
        for c in trip.cases_from(man, b.vref):
            if c['kind'] == 'software':
                continue                            # no software path without a CPU
            cid = f"L0-CPU-02.{c['sensor']}.{c['bound']}"
            what = f"CPU halted: {c['sensor']} {c['bound']} limit, {trip.path_text(c)}, crossed by a DAC write over SWD"
            if len(c['holds']) > (2 if wired else 1):
                line(cid, 'BLOCKED', what, 'outputs off by the hardware path alone', 'needs TP5-TP7 (two held pins)')
                results.append(dict(c, blocked='needs TP5-TP7'))
                continue
            r = halted_trip(b, c)
            results.append(r)
            if r.get('error'):
                line(cid, 'FAIL', what, 'outputs off by the hardware path alone', r['error'])
                continue
            halted = len(r['pcs']) == 2 and r['pcs'][0] == r['pcs'][1]
            on_before = bool(r['oenr_before']) and not (r['isr_before'] or 0) & r['flt_bit']
            tripped = bool((r['isr_after'] or 0) & r['flt_bit'])
            off = r['oenr_after'] == 0 and all(v == 0 for v in r['pins'].values())
            line(cid, judge(halted and on_before and tripped and off), what,
                 'CPU halted throughout; outputs on before; the path\'s FLT flag set; outputs disabled and every PWM '
                 'pin low',
                 f"PC {' = '.join(r['pcs'])}; OENR 0x{r['oenr_before'] or 0:03x} -> 0x{r['oenr_after'] or 0:03x}; "
                 f"FLT{c['path']['flt']} {'set' if tripped else 'NOT set'} (ISR 0x{r['isr_after'] or 0:x}); pins "
                 + ', '.join(f'{p}={v}' for p, v in r['pins'].items())
                 + f"; DAC {r['beyond_mV']:.0f} mV, threshold {c['threshold_mV']:.1f} mV")
    finally:
        b.close()
    return results, man, wired


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--part', choices=('1', '2', 'both'), default='both')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    report = {'limit_s': LOCKUP_LIMIT_S}
    if args.part in ('1', 'both'):
        scope = rigol_mho.RigolScope(rigol_mho.open_transport('/dev/usbtmc0'))
        report['scope'] = scope.idn()
        mux = MuxController(boards.find('mux', tries=10))
        mux.connect()
        Injector(mux).off()
        try:
            report['lockup'] = part1(args.out, scope, mux)
        finally:
            mux.disable_all()
            mux.close()
            scope.set_edge_trigger(1, 1.65, sweep='AUTO')
            scope.run()
            scope.close()
    if args.part in ('2', 'both'):
        report['halted'], man, report['tp5_tp7_wired'] = part2(args.out)
    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump(report, fh)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    man_out = {
        'title': 'L0-CPU · the protection with the CPU dead (SPOC bench)',
        'purpose': 'IEC 62109-1 Annex B route (a), ADR 0009: each protection must work with the CPU dead. A lockup '
                   'must turn every PWM output off within 1 µs through the HRTIM system fault (L0-CPU-01, also '
                   'L0-SUP-08.c); with the CPU halted, each hardware trip path must still turn the outputs off when '
                   'its threshold is crossed (L0-CPU-02).',
        'criteria': ['L0-CPU-'],
        'conditions': [
            ['01 firmware', 'SAFETY_SUP (tests/firmware/safety_supervision): bare Spin, PWM A, C, D, E, F at 200 kHz, '
                            '50 %; lockup = FAULTMASK set, then a fault; marker PA6 set a few cycles before'],
            ['01 capture', f'marker on SPOC CH1, the PWM pin on CH3 (injector off); one armed single trigger per pin '
                           f'(CH1 rising, {MARKER_TRIGGER_V} V, NORMal: PWM crosstalk on CH1 reaches ~1.9 V), '
                           f'{TIMEBASE_S * 1e6:g} µs/div; "last high" = the last sample '
                           'above 1.65 V after the marker'],
            ['02 firmware', 'SAFETY_L0_TWIST: production allocation of Twist 1.4.2 plus the reserved injection DAC'],
            ['02 CPU dead', 'halted over SWD after arming with the PWM on; DBGMCU: watchdog frozen in debug, HRTIM '
                            'running; the threshold crossed by writing DAC2 DHR12R1 over SWD '
                            f'({BEYOND_MV:g} mV past it); FLT flag, OENR and the PWM pins (GPIO IDR) read over SWD '
                            'before the CPU resumes'],
            ['Not covered here', 'software paths (they need the CPU); trip latency (C13); a real crash rather than a '
                                 'debug halt for 02 (a crash ends in a watchdog reset, covered by L0-SUP and C10)']],
        'target': [['Bench', 'SPOC: CUT Spin + MUX Spin soldered on the board, ST-Link on the CUT'],
                   ['MUX', f'Core-MUX ({trip.MUX_FW})'], ['Scope', report.get('scope', '')],
                   ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']],
        'report': 'report.json', 'figures': 'spoc_dead'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man_out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
