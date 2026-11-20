#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-EDG: edge cases of the hardware trips on the SPOC bench (ADR 0009, issue luizvilla/Core#23, C14).
##
## SAFETY_L0_TWIST on the CUT, set up as L0-TRP (tests/l0/spoc_trip): the CUT's DAC drives one current pin through
## CH1 -> TP1-TP3 -> CH2, the MUX injector holds or drives the other through CH3. Cases:
##   01 arming with a limit already exceeded is refused, and the outputs never turn on;
##   02 clearFault() is refused while the limit is still exceeded, accepted once it is back;
##   03 a second fault within the repeat window of a clear makes clearFault() refuse until a reboot (the second
##      fault from the MUX injector, the CUT's ThingSet being too slow for the 100 ms window of the test build);
##   04 two limits exceeded one after the other: both fault inputs recorded;
##   05 no trip just inside the thresholds, 10 s with the PWM on;
##   06 glitch filter of the comparator path: pulses on the second pin from the MUX injector's pulse mode (DAC source,
##      hold level -> 2.0 V -> hold level), each captured on SPOC CH3; the time above the threshold decides.
## Faults during dead time and trip latency need direct probes (C13).
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_edge/run.py --out DIR
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path[:0] = [os.path.join(REPO, 'tests', 'common'), os.path.join(REPO, 'tests', 'common', 'spoc')]
_spec = importlib.util.spec_from_file_location('spoc_trip_run', os.path.join(REPO, 'tests', 'l0', 'spoc_trip', 'run.py'))
trip = importlib.util.module_from_spec(_spec)   # tests/l0/spoc_trip/run.py
_spec.loader.exec_module(trip)

ENV, SHIELD = 'SAFETY_L0_TWIST', 'twist:1_4_2'
INSIDE_MV = 20.0          # case 05: this far inside each threshold
INSIDE_S = 10.0
REPEAT_CLEAR_MS = 100     # CONFIG_OWNTECH_SAFETY_REPEAT_CLEAR_MS of the test firmware (app.conf)
FILTER_NS = 282           # HRTIM fault filter of the comparator paths (fHRTIM/8, N = 6; safety_trips.c)
FILTER_MARGIN_NS = 40     # about one filter sample (47 ns) each side
PULSE_HIGH_MV = 2000
PULSE_WIDTHS_NS = list(range(60, 421, 20))   # requested; the scope measures what the pin got


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def fresh_bench(man):
    """A fresh boot (the arming self-tests run once per boot) and a connected bench."""
    trip.reset_cut()
    time.sleep(5)
    b = trip.Bench(man)
    b.vref = float(b.group('Inj')['rVref_mV'])
    return b


def comparator_case(man, sensor):
    return next(c for c in trip.cases_from(man, 2048.0) if c['sensor'] == sensor and c['kind'] == 'comparator')


def glitch_sweep(b, man, c1, c2, zero1):
    """One pulse per requested width on the second pin; returns the measured time above its threshold and the trip."""
    sc, sh = b.scope, b.mux._require_shell()
    thr = c2['threshold_mV'] / 1000
    out = []
    time.sleep(REPEAT_CLEAR_MS / 1000 + 0.5)
    ok, why, _ = b.arm(zero1, None)
    for w in PULSE_WIDTHS_NS:
        if not ok:
            out.append({'width_ns': w, 'error': why})
            break
        sc.set_timebase(200e-9, 0)
        for cmd in (':TRIGger:MODE EDGE', ':TRIGger:COUPling DC', ':TRIGger:EDGE:SOURce CHANnel3',
                    ':TRIGger:EDGE:SLOPe POSitive', f':TRIGger:EDGE:LEVel {thr - 0.2:.2f}',
                    ':TRIGger:SWEep NORMal', ':SINGle'):
            sc.write(cmd)
        time.sleep(0.6)
        sh.set_value('Mux/Inj/wPulseHigh_mV', str(PULSE_HIGH_MV))
        sh.set_value('Mux/Inj/wPulse_ns', str(w))
        sh.set_value('Mux/Inj/xPulse', 'true')
        t0 = time.time()
        while time.time() - t0 < 5 and not sc.query(':TRIGger:STATus?').strip().upper().startswith('STOP'):
            time.sleep(0.1)
        t, v = sc.waveform(3)
        above = [ti for ti, vi in zip(t, v) if vi > thr]
        st = b.safety()
        out.append({'width_ns': w, 'above_ns': round((above[-1] - above[0]) * 1e9) if above else 0,
                    'vmax': max(v), 'tripped': st['rTripped'] if st['rFault'] else 0, 'text': st['rText']})
        if st['rFault']:
            b.pwm(False)
            time.sleep(REPEAT_CLEAR_MS / 1000 + 0.2)
            b.cut.set_value('Safety/xClear', 'true')
            time.sleep(0.3)
            ok, why, _ = b.arm(zero1, None)
        time.sleep(0.2)
    b.pwm(False)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-flash', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    trip.production_manifest(SHIELD, args.out)
    man = trip.build_and_flash(ENV, args.out, flash=not args.no_flash)
    held = trip.held_sensors(man)
    if len(held) != 2:
        line('L0-EDG-01', 'BLOCKED', 'Edge cases', 'two held current pins', f'held: {held}')
        return 0
    s1, s2 = held                                   # Twist: I1_LOW (PA1), I2_LOW (PC1)
    rep = {}

    # 01 arming with a limit already exceeded
    b = fresh_bench(man)
    try:
        c1 = comparator_case(man, s1)
        b.route(c1['pin'], [s2])
        over = c1['threshold_mV'] + 50
        ok, why, st = b.arm(over, None)
        outputs_seen = st['rOutputs']
        b.pwm(False)
        rep['01'] = {'level_mV': over, 'armed': ok, 'why': why, 'state': st}
        line('L0-EDG-01', judge(not ok and outputs_seen == 0),
             f'Arming with {s1} already above its high limit ({over:.0f} mV, threshold {c1["threshold_mV"]:.0f} mV)',
             'arming refused, outputs never on', f"{'refused' if not ok else 'ARMED'}: {why or st['rText']}; "
             f"outputs 0x{outputs_seen:03x}")
    finally:
        b.close()

    # 02-05 on one fresh boot, armed first with both current pins at 0 A
    b = fresh_bench(man)
    try:
        armed, why = b.first_arming(held)
        if not armed:
            for n in ('02', '03', '04', '05'):
                line(f'L0-EDG-{n}', 'FAIL', 'Edge cases after the first arming', 'armed', why)
            return 1
        c1, c2 = comparator_case(man, s1), comparator_case(man, s2)
        b.route(c1['pin'], [s2])
        zero1 = trip.zero_mv(man, s1, b.vref)

        # 02 clear refused while exceeded
        ok, why, _ = b.arm(zero1, None)
        b.level(c1['threshold_mV'] + 50)
        time.sleep(0.2)
        tripped = b.safety()
        b.pwm(False)
        b.cut.set_value('Safety/xClear', 'true')
        time.sleep(0.3)
        st_refused = b.safety()
        b.level(zero1)
        time.sleep(0.3)
        b.cut.set_value('Safety/xClear', 'true')
        time.sleep(0.3)
        st_ok = b.safety()
        rep['02'] = {'tripped': tripped, 'refused': st_refused, 'after': st_ok}
        line('L0-EDG-02', judge(tripped['rFault'] == 3 and st_refused['rFault'] == 3 and st_refused['rClearRc'] == -1
                                and st_ok['rFault'] == 0 and st_ok['rClearRc'] == 0),
             f'clearFault() while {s1} is still above its limit, then once it is back at 0 A',
             'refused (-1), fault kept; then accepted (0)',
             f"tripped: {tripped['rText']}; clear while exceeded: rc {st_refused['rClearRc']}, fault "
             f"{st_refused['rFault']}; clear at 0 A: rc {st_ok['rClearRc']}, fault {st_ok['rFault']}")

        # 04 two limits exceeded one after the other (before 03, which ends in "reboot required")
        time.sleep(REPEAT_CLEAR_MS / 1000 + 0.5)
        ok, why, _ = b.arm(zero1, None)
        b.level(c1['threshold_mV'] + 50)
        time.sleep(0.2)
        b.inj.level(int(round(c2['threshold_mV'] + 50)))   # same source: CH3 stays routed to the held pin
        time.sleep(0.3)
        st = b.safety()
        bits = {f"FLT{c1['path']['flt']}": c1['path']['flt'], f"FLT{c2['path']['flt']}": c2['path']['flt']}
        got = {k: bool(st['rTripped'] & (1 << (v - 1))) for k, v in bits.items()}
        rep['04'] = {'state': st, 'flags': got}
        line('L0-EDG-04', judge(all(got.values()) and st['rOutputs'] == 0),
             f'{s1} then {s2} above their high limits: both fault inputs recorded',
             ' and '.join(bits) + ' in the tripped mask, outputs off',
             f"tripped 0x{st['rTripped']:02x} ({', '.join(f'{k} {chr(10003) if v else chr(10007)}' for k, v in got.items())}); "
             f"{st['rText']}")
        b.pwm(False)
        b.inj.level(int(round(trip.zero_mv(man, s2, b.vref))))
        b.level(zero1)
        time.sleep(0.5)
        b.cut.set_value('Safety/xClear', 'true')
        time.sleep(0.3)

        # 05 no trip just inside the thresholds, 10 s each
        quiet = []
        for c, src in ((c1, 'dac'),):
            for bound in ('high', 'low'):
                cc = next(x for x in trip.cases_from(man, b.vref) if x['sensor'] == c['sensor'] and x['bound'] == bound
                          and x['kind'] != 'software')
                level = cc['threshold_mV'] - INSIDE_MV if bound == 'high' else cc['threshold_mV'] + INSIDE_MV
                time.sleep(REPEAT_CLEAR_MS / 1000 + 0.5)
                ok, why, _ = b.arm(zero1, None)
                b.level(level)
                t0 = time.time()
                faults = 0
                while time.time() - t0 < INSIDE_S:
                    st = b.safety()
                    faults += st['rFault'] != 0
                    if st['rFault']:
                        break
                    time.sleep(0.5)
                b.pwm(False)
                b.level(zero1)
                quiet.append({'sensor': cc['sensor'], 'bound': bound, 'level_mV': level,
                              'threshold_mV': cc['threshold_mV'], 'fault': faults > 0, 'text': st['rText']})
                if faults:
                    time.sleep(0.3)
                    b.cut.set_value('Safety/xClear', 'true')
        rep['05'] = quiet
        for q in quiet:
            line(f"L0-EDG-05.{q['sensor']}.{q['bound']}", judge(not q['fault']),
                 f"{q['sensor']} {INSIDE_MV:g} mV inside its {q['bound']} threshold for {INSIDE_S:g} s, PWM on",
                 'no trip', f"{q['level_mV']:.0f} mV against {q['threshold_mV']:.1f} mV: "
                 + ('TRIPPED: ' + q['text'] if q['fault'] else 'no trip'))

        # 06 glitch filter of the comparator path (second pin, from the MUX injector's pulse mode)
        rep['06'] = glitch_sweep(b, man, c1, c2, zero1)
        thr2 = c2['threshold_mV'] / 1000
        bit = 1 << (c2['path']['flt'] - 1)
        rows = [r for r in rep['06'] if r.get('above_ns') is not None]
        short = [r for r in rows if r['above_ns'] <= FILTER_NS - FILTER_MARGIN_NS]
        long_ = [r for r in rows if r['above_ns'] >= FILTER_NS + FILTER_MARGIN_NS]
        quiet = [r['above_ns'] for r in rows if not r['tripped'] & bit]
        trips = [r['above_ns'] for r in rows if r['tripped'] & bit]
        ok = (bool(short) and bool(long_) and all(not r['tripped'] & bit for r in short)
              and all(r['tripped'] & bit for r in long_))
        line('L0-EDG-06', judge(ok),
             f"{s2} comparator (COMP{c2['path']['comp']} -> FLT{c2['path']['flt']}) "
             f"against pulses above its {c2['threshold_mV']:.0f} mV threshold (filter {FILTER_NS} ns)",
             f'no trip up to {FILTER_NS - FILTER_MARGIN_NS} ns above the threshold, a trip from '
             f'{FILTER_NS + FILTER_MARGIN_NS} ns',
             f"{len(rows)} pulses; longest without a trip {max(quiet) if quiet else 'none'} ns, shortest with one "
             f"{min(trips) if trips else 'none'} ns (time above {thr2:.2f} V on CH3)"
             + ('' if all(not (r['tripped'] & ~bit & 0x3F) for r in rows) else
                '; the watchdog caught some pulses too (no filter, sampled every PWM period)'))

        # 03 second fault within the repeat window of a clear: trip 1 on the first pin (CUT DAC), trip 2 on the
        # second pin from the MUX injector right after the clear returns (two writes on the MUX's own port: the
        # CUT's ThingSet alone takes more than the 100 ms window)
        time.sleep(REPEAT_CLEAR_MS / 1000 + 0.5)
        zero2 = int(round(trip.zero_mv(man, s2, b.vref)))
        b.level(c1['threshold_mV'] + 50)            # trip 1 (armed trips act with the PWM off too)
        time.sleep(0.3)
        b.level(zero1)
        time.sleep(0.3)
        mux_sh = b.mux._require_shell()
        mux_sh.set_value('Mux/Inj/wLevel_mV', str(int(round(c2['threshold_mV'] + 50))))   # loaded, not applied
        times = {}

        def second_fault():
            time.sleep(0.02)
            times['sent'] = time.time()
            mux_sh.set_value('Mux/Inj/xApply', 'true')   # trip 2
            times['done'] = time.time()
        th = threading.Thread(target=second_fault)
        t_clear = time.time()
        th.start()
        b.cut.set_value('Safety/xClear', 'true')
        dt_clear_ms = (time.time() - t_clear) * 1e3
        th.join()
        dt_ms = (times['done'] - t_clear) * 1e3
        time.sleep(0.1)
        st2 = b.safety()                              # the second fault latched?
        time.sleep(0.3)
        b.inj.level(zero2)
        time.sleep(0.3)
        b.cut.set_value('Safety/xClear', 'true')
        time.sleep(0.3)
        st = b.safety()
        rep['03'] = {'second_fault_after_ms': dt_ms, 'clear_reply_ms': dt_clear_ms, 'after_second_fault': st2,
                     'state': st}
        # The ADR has two readings (2026-11-20, open): decision text "a repeated clear within N s needs a reboot"
        # (what safety_supervision_clear_fault() does: it compares the new clear with the previous one) and the
        # implementation note / Kconfig "a second fault within N s of a clear". Recorded, not judged.
        line('L0-EDG-03', 'INFO',
             f'A second fault at most {dt_ms:.0f} ms after a clear, then a clear about 0.6 s later (repeat window '
             f'{REPEAT_CLEAR_MS} ms in the test build)',
             'ADR open: "repeated clear within N s" (the code) or "second fault within N s of a clear" (Kconfig, '
             'ADR implementation note)',
             f"second fault latched: {st2['rText'] or 'NO'}; second clear: rc {st['rClearRc']}, fault {st['rFault']}"
             + (' (accepted: the code counts clear-to-clear time, so the fault inside the window is not counted)'
                if st['rClearRc'] == 0 else f": {st['rText']}"))
    finally:
        b.close()
    trip.reset_cut()                                   # leave a fresh boot (03 needs a reboot to clear)

    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump(rep, fh)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    man_out = {
        'title': 'L0-EDG · edge cases of the hardware trips (SPOC bench)',
        'purpose': 'Arming, clearing and reporting at the edges (ADR 0009): a limit already exceeded at arming, a clear '
                   'while still exceeded, a repeated fault after a clear (IEC 62109-1 4.4.2.2), two faults at once, '
                   'and quiet operation just inside the thresholds.',
        'criteria': ['L0-EDG-'],
        'conditions': [
            ['Firmware', f'{ENV}: production allocation of {SHIELD} plus the reserved injection DAC; repeat-clear '
                         f'window {REPEAT_CLEAR_MS} ms (CONFIG_OWNTECH_SAFETY_REPEAT_CLEAR_MS, 10 s in production)'],
            ['Stimulus', 'the CUT DAC (PA6) on the first current pin through CH1 -> TP1-TP3 -> CH2; the MUX injector '
                         '(DAC source) holds or drives the second through CH3'],
            ['Boots', '01 on a fresh boot (the arming self-tests run once per boot); 02-05 on a second fresh boot, '
                      'armed first with both current pins at 0 A; a reset at the end (03 leaves "reboot required")'],
            ['Glitches (06)', f'MUX injector pulse mode (Core-MUX mux-injector, fe04fe3): the DAC '
                              f'source steps the second pin from its 0 A hold to {PULSE_HIGH_MV} mV and back; '
                              'requested widths ' + f'{PULSE_WIDTHS_NS[0]}-{PULSE_WIDTHS_NS[-1]} ns; each pulse captured on '
                              'CH3 (200 ns/div); the time above the threshold decides'],
            ['Not covered here', 'a fault during dead time, trip latency: direct probes (C13)']],
        'target': [['Bench', 'SPOC: CUT Spin + MUX Spin, ST-Link on the CUT'], ['MUX', f'Core-MUX ({trip.MUX_FW})'],
                   ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']],
        'report': 'report.json', 'figures': 'spoc_edge'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man_out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
