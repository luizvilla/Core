#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-TRP: every protection path of the board's manifest trips at its threshold, on the SPOC bench
##        (ADR 0009, issue luizvilla/Core#23, C11).
##
## The cases come from the protection manifest of the test build: one per limit, bound and path (comparator, ADC
## watchdog, software). The build is the production allocation plus the reserved injection DAC (L0-TRP-01 compares
## the two manifests). For each case the CUT's own DAC (DAC2_CH1 on PA6, /Inj) drives the sensor pin under test
## through SPOC CH1 -> TP1-TP3 -> CH2; the current pins whose low bound is an ADC watchdog are held at their 0 A
## level by the MUX injector through CH3 and, with TP5-TP7 wired, CH4 (a floating current pin fails the arming
## self-tests). Routes change only with the PWM stopped. Per case: arm at a start level inside the threshold and
## check it stays quiet, ramp 10 mV per step until the trip, recover, ramp 1 mV per step from just before that
## level; the fine trip level is judged against the threshold computed from the manifest's counts.
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_trip/run.py --out DIR --env SAFETY_L0_TWIST --shield twist:1_4_2
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
COMMON = os.path.join(REPO, 'tests', 'common')
sys.path[:0] = [COMMON, os.path.join(COMMON, 'spoc')]
import serial  # noqa: E402
import rigol_mho  # noqa: E402
from spoc import boards  # noqa: E402
from spoc.injector import Injector  # noqa: E402
from mux_controller import MuxController, PIN_TO_ROUTE  # noqa: E402
from thingset import ThingSetShell  # noqa: E402

PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
MUX_FW = 'owntech-foundation/SPOC_Software mux-injector fe04fe3'
SOURCE_PIN = 'PA6'           # DAC2_CH1, reserved from the allocation (app.overlay)
TOL_MV = 25.0                # trip level against the threshold; DAC, ADC and comparators share the CUT's VREFBUF
START_MV = 150.0             # start level inside the threshold
COARSE_MV, FINE_MV = 10.0, 1.0
BEYOND_MV = 100.0            # how far past the threshold a ramp goes before it gives up
SETTLE_S = 0.03
QUIET_S = 0.2
TRIP_FAULT = 3               # SAFETY_FAULT_TRIP
HOLD_SPREAD_MV = 50.0        # held pins whose 0 A levels differ by less than this share the injector's mean level


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


# ---------------------------------------------------------------- manifests

def production_manifest(shield, out):
    """Production allocation of the shield (tools/safety_check.py on an empty draft)."""
    draft = os.path.join(out, 'empty_draft.overlay')
    with open(draft, 'w') as fh:
        fh.write('/* empty: the production shield as it is (L0-TRP-01) */\n')
    path = os.path.join(out, 'production_manifest.json')
    r = subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'safety_check.py'), draft, '--base', shield,
                        '--json', path], cwd=REPO, capture_output=True, text=True)
    if not os.path.exists(path):
        raise RuntimeError(f'safety_check failed: {r.stdout[-1500:]}{r.stderr[-1500:]}')
    return json.load(open(path))


def build_and_flash(env, out, flash=True):
    """Builds (and flashes) the test env; returns its protection manifest. Runs after safety_check.py, whose
    environment change makes PlatformIO wipe every build folder."""
    r = subprocess.run([PIO, 'run', '-e', env] + (['-t', 'upload'] if flash else []), cwd=REPO,
                       capture_output=True, text=True)
    subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
    with open(os.path.join(out, f'build_{env}.log'), 'w') as fh:
        fh.write(r.stdout[-20000:] + r.stderr[-20000:])
    if r.returncode != 0 or (flash and 'Verified OK' not in r.stdout + r.stderr):
        raise RuntimeError(f'build or flash of {env} failed (build_{env}.log)')
    man = os.path.join(REPO, '.pio', 'build', env, 'zephyr', 'safety', 'protection-manifest.json')
    return json.load(open(man))


def reset_cut():
    ocd = os.path.expanduser('~/.platformio/packages/tool-openocd')
    subprocess.run([os.path.join(ocd, 'bin', 'openocd'), '-d0', '-s', os.path.join(ocd, 'openocd', 'scripts'),
                    '-f', 'interface/stlink.cfg', '-f', 'target/stm32g4x.cfg', '-c', 'init; reset run; exit'],
                   capture_output=True, timeout=30)


def manifest_diff(prod, test):
    """Entries of the test build's manifest that differ from production, the reserved DACs aside."""
    p, t = json.loads(json.dumps(prod)), json.loads(json.dumps(test))
    for m in (p, t):
        m['model'].pop('reserved', None)
        m.pop('warnings', None)
    diffs = []
    for k in sorted(set(p) | set(t)):
        if p.get(k) != t.get(k):
            diffs.append(k)
    return diffs


# ---------------------------------------------------------------- the cases

def sensor_pin(man, sensor, path):
    chans = man['model']['sensors'][sensor]['channels']
    if path.get('kind') == 'comparator':
        return path['mcu_pin']
    if path.get('kind') == 'adc_watchdog':
        for c in chans:
            if c['adc'] == path['adc'] and c['channel'] == path['channel']:
                return c['mcu_pin']
    default = man['model']['sensors'][sensor]['default_adc']
    for c in chans:
        if c['adc'] == default:
            return c['mcu_pin']
    return chans[0]['mcu_pin']


def threshold_mv(lim, bound, path, vref):
    """Pin voltage at which the path trips, from the manifest's counts (12-bit scale of the CUT's VREFBUF)."""
    lsb = vref / 4096.0
    kind = path['kind']
    if kind == 'comparator':
        return path['dac_code'] * lsb
    if kind == 'adc_watchdog':
        c = path['counts_high'] if bound == 'high' else path['counts_low']
        if path['bits'] == 8:   # the watchdog compares raw >> 4: trips at raw >= (HT + 1) * 16 or raw < LT * 16
            return ((c + 1) * 16 if bound == 'high' else c * 16) * lsb
        return (c + 0.5 if bound == 'high' else c - 0.5) * lsb
    raw = (lim[bound] - lim['offset']) / lim['gain']   # software: value = raw * gain + offset
    return raw * lsb


def held_sensors(man):
    """Sensors whose low bound is an ADC watchdog: a floating pin fails their arming self-tests."""
    return [l['sensor'] for l in man['limits'] if l['paths']['low'].get('kind') == 'adc_watchdog']


def zero_mv(man, sensor, vref):
    s = man['model']['sensors'][sensor]
    return (-s['offset'] / s['gain']) * vref / 4096.0


def cases_from(man, vref):
    held = held_sensors(man)
    out = []
    for lim in man['limits']:
        for bound in ('high', 'low'):
            path = lim['paths'][bound]
            if path.get('kind') not in ('comparator', 'adc_watchdog', 'software'):
                continue
            pin = sensor_pin(man, lim['sensor'], path)
            lsb = vref / 4096.0
            window_mv = abs(lim['high'] - lim['low']) / lim['gain'] * lsb    # the sensor's limits, in pin mV
            out.append({'sensor': lim['sensor'], 'bound': bound, 'kind': path['kind'], 'path': path, 'pin': pin,
                        'limit': lim[bound], 'threshold_mV': threshold_mv(lim, bound, path, vref),
                        'start_offset_mV': min(START_MV, 0.4 * window_mv),
                        'holds': [h for h in held if h != lim['sensor']]})
    return out


BOARD = 'twist'   # set by main() from --shield: each board's results keep their own IDs in the control center


def case_id(c):
    return f"{'L0-TRP-03' if c['kind'] == 'software' else 'L0-TRP-02'}.{BOARD}.{c['sensor']}.{c['bound']}"


def path_text(c):
    p = c['path']
    if c['kind'] == 'comparator':
        return f"COMP{p['comp']} -> FLT{p['flt']}"
    if c['kind'] == 'adc_watchdog':
        return f"{p['adc'].upper()} AWD{p['awd']} -> FLT{p['flt']}"
    return 'software check'


# ---------------------------------------------------------------- bench

class Bench:
    def __init__(self, man):
        self.man = man
        self.scope = rigol_mho.RigolScope(rigol_mho.open_transport('/dev/usbtmc0'))
        self.idn = self.scope.idn()
        ports, t0 = {}, time.time()   # USB re-enumerates after a flash or a reset: up to 60 s for both
        while time.time() - t0 < 60 and len(ports) < 2:
            for r in ('mux', 'cut'):
                ports[r] = ports.get(r) or boards.find(r, tries=1)
            ports = {r: p for r, p in ports.items() if p}
        missing = [r for r in ('mux', 'cut') if r not in ports]
        if missing:
            raise RuntimeError(f'no ThingSet answer from: {", ".join(missing)}')
        self.mux = MuxController(ports['mux'])
        self.mux.connect()
        self.inj = Injector(self.mux)
        self.ser = serial.Serial(ports['cut'], 115200, timeout=0.2, write_timeout=1.0)
        self.cut = ThingSetShell(self.ser)
        self.cut.enter_thingset()
        units = 0
        for leg in man['legs']:
            units |= 1 << 'abcdef'.index(leg['timer'][-1])
        self.units = units or 1

    def close(self):
        try:
            self.pwm(False)
            self.cut_set('Inj', wOn='false')
            self.cut_set('Sw', wName='')
            self.mux.disable_all()
            self.inj.off()
        finally:
            self.ser.close()
            self.mux.close()
            self.scope.close()

    def group(self, name):
        raw = self.cut.send_cmd('?' + name, read_timeout=1.0).decode(errors='replace')
        return json.loads(raw[raw.find('{'):raw.rfind('}') + 1])

    def cut_set(self, group, **items):
        for k, v in items.items():
            ok, resp = self.cut.set_value(f'{group}/{k}', str(v))
            if not ok:
                raise RuntimeError(f'ThingSet write {group}/{k}={v} failed: {resp!r}')
        self.cut.set_value(f'{group}/xApply', 'true')

    def level(self, mv):
        self.cut_set('Inj', wLevel_mV=int(round(mv)), wOn='true')

    def pwm(self, on):
        if on:
            self.cut_set('Pwm', wFreq_Hz=200000, wDuty=0.5, wUnits=self.units, wRun='true')
        else:
            self.cut_set('Pwm', wRun='false')

    def wire_tp5_tp7(self):
        """True when TP5 and TP7 are joined: the injector's level shows on scope CH4 with CH4 routed nowhere."""
        self.mux.disable_all()
        self.inj.level(1200)
        time.sleep(0.5)
        v4 = self.scope.measure('VAVG', 4)
        self.inj.off()
        return v4 is not None and abs(v4 - 1.2) < 0.15, v4

    def route(self, target, holds):
        """Routes with the PWM stopped: injector source first (a source change disables CH3), then CH3/CH4 to the
        held pins, CH1 to the DAC pin and CH2 to the pin under test."""
        self.pwm(False)
        hold_mv = [zero_mv(self.man, h, self.vref) for h in holds]
        if holds:
            self.inj.level(int(round(sum(hold_mv) / len(hold_mv))))   # one injector: the mean of the 0 A levels
        else:
            self.inj.off()
        self.mux.disable_all()
        sh = self.mux._require_shell()
        plan = [('ch1', SOURCE_PIN), ('ch2', target)] + list(zip(('ch3', 'ch4'), [self.pin_of(h) for h in holds]))
        for ch, pin in plan:
            r = PIN_TO_ROUTE[pin]
            self.mux._set_mux_group(sh, self.mux._mux_path(r['mux'], ch), r['input_index'], True)
        return plan

    def pin_of(self, sensor):
        lim = next(l for l in self.man['limits'] if l['sensor'] == sensor)
        return sensor_pin(self.man, sensor, lim['paths']['low'])

    def safety(self):
        return self.group('Safety')

    def first_arming(self, held):
        """Arms once per boot with every held current pin at its 0 A level: the arming self-tests (window, idle
        plausibility) run at the first data start only, and the trips then stay armed until the next reset."""
        if not held:
            ok, why, _ = self.arm(1000.0, None)
            self.pwm(False)
            return ok, why
        self.route(self.pin_of(held[0]), held[1:])
        ok, why, _ = self.arm(zero_mv(self.man, held[0], self.vref), None)
        self.pwm(False)
        return ok, why

    def arm(self, start_mv, sw_sensor):
        """Start level, software watch, clear, PWM on; returns (ok, why, safety state after the quiet time)."""
        self.pwm(False)
        self.level(start_mv)
        self.cut_set('Sw', wName=sw_sensor or '')
        time.sleep(0.2)
        st = self.safety()
        if st['rFault']:
            self.cut.set_value('Safety/xClear', 'true')
            time.sleep(0.2)
            st = self.safety()
            if st['rFault']:
                return False, f'clear refused ({st["rClearRc"]}): {st["rText"]}', st
        self.pwm(True)
        time.sleep(QUIET_S)
        st = self.safety()
        if not st['rArmed'] or not st['rOutputs']:
            return False, f'arming refused or outputs off: {st["rText"] or "rc " + str(self.group("Pwm")["rRc"])}', st
        if st['rFault']:
            return False, f'tripped at the start level: {st["rText"]}', st
        return True, '', st

    def ramp(self, start, stop, step):
        """Steps from start towards stop; returns (trip level or None, last quiet level, trace, safety state)."""
        sign = 1 if stop > start else -1
        trace, last_quiet, v = [], start, start
        while (v - stop) * sign <= 0:
            self.level(v)
            time.sleep(SETTLE_S)
            st = self.safety()
            trace.append([round(v, 1), int(bool(st['rFault']))])
            if st['rFault']:
                return v, last_quiet, trace, st
            last_quiet = v
            v += sign * step
        return None, last_quiet, trace, self.safety()


def run_case(b, c):
    high = c['bound'] == 'high'
    thr = c['threshold_mV']
    sign = 1 if high else -1
    start = min(max(thr - sign * c['start_offset_mV'], 20.0), b.vref - 20.0)   # inside both bounds of a narrow window
    stop = min(max(thr + sign * BEYOND_MV, 0.0), b.vref)
    res = dict(c, start_mV=start, routes=b.route(c['pin'], c['holds']))
    sw = c['sensor'] if c['kind'] == 'software' else None
    ok, why, _ = b.arm(start, sw)
    if not ok:
        res.update(error=why)
        return res
    coarse, _, tr_c, _ = b.ramp(start, stop, COARSE_MV)
    res['coarse_trace'] = tr_c
    if coarse is None:
        res.update(error=f'no trip up to {stop:.0f} mV')
        b.pwm(False)
        return res
    ok, why, _ = b.arm(start, sw)
    if not ok:
        res.update(error=f'second arming: {why}', coarse_mV=coarse)
        return res
    fine_start = coarse - sign * (COARSE_MV + 5 * FINE_MV)
    b.level((start + fine_start) / 2)
    time.sleep(SETTLE_S)
    fine, quiet, tr_f, st = b.ramp(fine_start, stop, FINE_MV)
    b.pwm(False)
    flt = c['path'].get('flt')
    res.update(coarse_mV=coarse, fine_mV=fine, last_quiet_mV=quiet, fine_trace=tr_f, state=st,
               expected_flt=flt)
    if fine is not None:
        res['error_mV'] = fine - thr
    return res


def verdict(c):
    """(PASS/FAIL, measured text) for one case."""
    if c.get('error'):
        return 'FAIL', c['error']
    if c.get('fine_mV') is None:
        return 'FAIL', f"coarse trip at {c['coarse_mV']:.0f} mV, none on the fine ramp"
    st = c['state']
    tripped = st.get('rTripped', 0)
    text = st.get('rText', '')
    checks = [st['rFault'] == TRIP_FAULT, st['rOutputs'] == 0, abs(c['error_mV']) <= TOL_MV]
    if c['kind'] == 'software':
        checks.append(c['sensor'] in text and 'software' in text.lower())
        path_ok = 'software trip'
    else:
        checks.append(bool(tripped & (1 << (c['expected_flt'] - 1))))
        path_ok = f"FLT{c['expected_flt']} {'set' if checks[-1] else 'NOT set'} (tripped 0x{tripped:02x})"
    measured = (f"trip at {c['fine_mV']:.0f} mV, threshold {c['threshold_mV']:.1f} mV "
                f"({c['error_mV']:+.1f} mV); {path_ok}; outputs {'off' if st['rOutputs'] == 0 else 'ON'}; {text}")
    return judge(all(checks)), measured


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--env', default='SAFETY_L0_TWIST')
    ap.add_argument('--shield', default='twist:1_4_2')
    ap.add_argument('--no-flash', action='store_true', help='the CUT already runs the test build')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    global BOARD
    BOARD = args.shield.split(':')[0]

    prod = production_manifest(args.shield, args.out)
    man = build_and_flash(args.env, args.out, flash=not args.no_flash)
    reset_cut()   # a fresh boot: the arming self-tests run once, at the first data start, with the pins held
    time.sleep(5)
    with open(os.path.join(args.out, 'test_manifest.json'), 'w') as fh:
        json.dump(man, fh, indent=1)
    diffs = manifest_diff(prod, man)
    reserved = man['model'].get('reserved', {}).get('dacs', [])
    line(f'L0-TRP-01.{BOARD}.a', judge(not diffs and reserved == ['DAC2_CH1']),
         f'Test build {args.env}: protection manifest against the production {args.shield} allocation',
         'identical but for reserved DAC2_CH1',
         'identical, reserved ' + ', '.join(reserved) if not diffs else 'differs in: ' + ', '.join(diffs))
    if diffs:
        for c in cases_from(man, 2048.0):
            line(case_id(c), 'BLOCKED', f"{c['sensor']} {c['bound']} limit, {path_text(c)}", 'trip at the threshold',
                 'the test build differs from production (L0-TRP-01)')
        return 1

    b = Bench(man)
    results = []
    try:
        b.vref = float(b.group('Inj')['rVref_mV'])
        wired, v4 = b.wire_tp5_tp7()
        cases = cases_from(man, b.vref)
        holds_ok = 2 if wired else 1
        held = held_sensors(man)
        if len(held) - 1 > holds_ok:
            armed, why = False, f'{len(held)} current pins to hold ({", ".join(held)}), {holds_ok + 1} sources'
        else:
            armed, why = b.first_arming(held)
        line(f'L0-TRP-01.{BOARD}.b', judge(armed), 'First arming of the boot, held current pins at 0 A: self-tests pass',
             'armed, outputs on, quiet', 'armed' if armed else why)
        for c in cases:
            what = f"{c['sensor']} {c['bound']} limit {c['limit']:g}, {path_text(c)}, pin {c['pin']}"
            if len(c['holds']) > holds_ok:
                why = (f"needs {len(c['holds'])} held pins ({', '.join(c['holds'])}); TP5-TP7 not wired "
                       f"(scope CH4 read {v4 if v4 is None else round(v4, 3)} V with the injector at 1.2 V)")
                line(case_id(c), 'BLOCKED', what, f'trip within ±{TOL_MV:g} mV of the threshold', why)
                results.append(dict(c, blocked=why))
                continue
            levels = [zero_mv(man, h, b.vref) for h in c['holds']]
            if levels and max(levels) - min(levels) > HOLD_SPREAD_MV:
                why = (f"held pins need 0 A levels {', '.join(f'{x:.0f}' for x in levels)} mV, more than "
                       f"{HOLD_SPREAD_MV:g} mV apart: one injector holds them")
                line(case_id(c), 'BLOCKED', what, f'trip within ±{TOL_MV:g} mV of the threshold', why)
                results.append(dict(c, blocked=why))
                continue
            r = run_case(b, c)
            v, measured = verdict(r)
            r['verdict'] = v
            results.append(r)
            line(case_id(c), v, what, f'trip fault on its path, outputs off, within ±{TOL_MV:g} mV of '
                 f"{c['threshold_mV']:.1f} mV", measured)
        snapshot_idn = b.idn
    finally:
        b.close()

    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump({'vref_mV': b.vref, 'tp5_tp7_wired': wired, 'tolerance_mV': TOL_MV, 'cases': results}, fh)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    man_out = {
        'title': f'L0-TRP · protection paths trip at their thresholds, {args.shield} (SPOC bench)',
        'purpose': 'Every limit, bound and path of the board\'s protection manifest is driven across its threshold on '
                   'the CUT Spin (ADR 0009): the trip must come within the tolerance of the threshold computed from '
                   'the manifest, on the path\'s own fault input (or the software check), with the outputs off.',
        'criteria': ['L0-TRP-'],
        'conditions': [
            ['Firmware', f'{args.env} (tests/firmware/safety_l0): production allocation of {args.shield} plus the '
                         'reserved injection DAC; checked by L0-TRP-01'],
            ['Stimulus', 'CUT DAC2_CH1 on PA6, buffered, through SPOC CH1 -> TP1-TP3 -> CH2 to the pin under test; '
                         f'full scale {b.vref:.0f} mV (the CUT\'s VREFBUF, shared with its ADCs and comparator DACs)'],
            ['Held pins', 'current pins whose low bound is an ADC watchdog, at their 0 A level, by the MUX injector '
                          'through CH3 and, with TP5-TP7 wired, CH4 (a floating current pin fails the arming '
                          f'self-tests); TP5-TP7 wired: {"yes" if wired else "no"}'],
            ['Ramp', f'start {START_MV:g} mV inside the threshold, quiet for {QUIET_S * 1e3:.0f} ms armed; '
                     f'{COARSE_MV:g} mV steps to the trip, recover, {FINE_MV:g} mV steps from 15 mV before it; '
                     f'{SETTLE_S * 1e3:.0f} ms per step'],
            ['PWM', '200 kHz, duty 0.5, the legs of the manifest; control task 100 µs (feeds the watchdog, read-back)'],
            ['Software paths', 'only the sensor under test is watched (/Sw), so the floating pins of the bare Spin do '
                               'not trip the others'],
            ['Tolerance', f'±{TOL_MV:g} mV; an 8-bit watchdog has 16-count (8 mV) steps'],
            ['Not covered here', 'trip latency (C13), glitch filters and edge cases (C14), CPU-dead cases (C12)']],
        'target': [['Bench', 'SPOC: CUT Spin + MUX Spin soldered on the board'],
                   ['CUT', f'Spin 1.2.0 with {args.env}, ThingSet port {boards.find("cut")}, ST-Link'],
                   ['MUX', f'Core-MUX ({MUX_FW}), port {boards.find("mux")}'],
                   ['Scope', snapshot_idn], ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']],
        'report': 'report.json', 'figures': 'spoc_trip'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man_out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
