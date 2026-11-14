#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-API-02..06: one Safety API on the Spin through an ST-Link (ADR 0009, issue luizvilla/Core#22).
##
## Builds and flashes env SAFETY_API (production), runs every case over SWD, then the same with env SAFETY_API_DEV
## (development mode, same sources), judges, and writes the test-case folder. Bare Spin, Twist 1.4.1 devicetree.
## It REFLASHES the board: flash env USB again before any bench session.
##
## @par Usage
##   python3 tests/l0/safety_api/run.py --out DIR [--no-flash]
import argparse
import json
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'common'))
import target_harness as th  # noqa: E402

ENVS = ('SAFETY_API', 'SAFETY_API_DEV')
N = 26
FIELDS = ['magic', 'layout', 'boot_count', 'ack', 'busy', 'latched_at_boot', 'pending'] + [f'r{i}' for i in range(N)]
# REACTION needs the PWM not yet initialised and DEV the PWM never started (disarm): both run first after the reset.
# SW_TRIP reboots (data acquisition started by the scheduler): it runs last.
CASES = [('CLEAR', 0), ('REACTION', 1), ('DEV', 2), ('WRAPPERS', 3), ('NVS', 4), ('SW_TRIP', 5)]
FIRST_AFTER_REBOOT = 15        # results R_SW.. are written after SW_TRIP's reboot
V_HIGH = 3                     # sensor_t on the Twist (UNDEFINED 0, V1_LOW 1, V2_LOW 2, V_HIGH 3)
TASK_PERIOD_US = 100
SW_REACTION_US = 7 * TASK_PERIOD_US   # declared 5 control periods (ADR 0009), +1 for the period in progress, +1 margin


def f(bits):
    return struct.unpack('<f', struct.pack('<I', bits & 0xFFFFFFFF))[0]


def s8(v):
    v &= 0xFF
    return v - 256 if v > 127 else v


def run_env(env, out, flash):
    if flash:
        print(f'== build and flash env {env}', flush=True)
        rc = th.flash(env, os.path.join(out, f'build_{env}.txt'))
        if rc != 0:
            return None, '', [{'case': 'build', 'status': f'exit {rc}'}]
    p = th.Protocol(env, 'g_safety_api_report', 'g_safety_api_cmd', FIELDS, 0x31495041)
    th.openocd('reset run', check=False)   # a fresh boot: REACTION and DEV need the PWM untouched
    entries, before, rep = [], None, None
    for name, code in CASES:
        if name == 'SW_TRIP':
            before = rep
        rep, entry = p.run(name, code)
        entries.append(entry)
    if rep is None or before is None:
        return None, '', entries
    r = [before[f'r{i}'] for i in range(FIRST_AFTER_REBOOT)] + [rep[f'r{i}'] for i in range(FIRST_AFTER_REBOOT, N)]
    tw = th.read_words(th.symbol(env, 'g_safety_api_text')[0], 32) or []
    text = b''.join(w.to_bytes(4, 'little') for w in tw).split(b'\0')[0].decode(errors='replace')
    return r, text, entries


def judge_common(tag, r, text, dev):
    """Criteria true in both builds: the software check reacts like every other trip."""
    J, L = th.judge, th.line
    hz = r[22] or 170_000_000
    sw, reason, flags = r[15], r[16], r[18]
    fault_after = (sw >> 24) & 0xFF
    value = f(r[17])
    L(f'{tag}.a', J(s8(sw) == 0), 'Software min of V_HIGH tightened to 99 (devicetree 10)', 'rc 0', f'rc {s8(sw)}')
    L(f'{tag}.b', J(fault_after == 3 and reason == (0x400 | V_HIGH) and value < 99.0),
      'V_HIGH below its software min: a TRIP is latched with the sensor, the bound and the reading',
      f'fault 3 (TRIP), detail {0x400 | V_HIGH:#x} (software, V_HIGH, low)',
      f'fault {fault_after}, detail {reason:#x}, reading {value:.2f}')
    mask = 0xFF3
    pins, expect, path = r[19] & mask, r[20] & mask, (flags >> 8) & 3
    L(f'{tag}.c', J(flags & 1 and flags & 2 and pins == expect and path == 1),
      'Stopped like any trip: through a forced HRTIM fault (every leg in its safe state), safe output dropped, no GPIO '
      'reconfiguration', 'outputs off, pins = fault state, relay open, path external event',
      f'outputs off {flags & 1}, pins {pins:#05x} (fault state {expect:#05x}), relay open {(flags >> 1) & 1}, '
      f"path {['none', 'external event', 'comparator'][path]}")
    L(f'{tag}.d', J(flags & 4 and flags & 8 and flags & 16),
      'PWM refused while latched; clearFault() accepted once the PWM is off and the check released; PWM back',
      'refused, cleared, back', f'{(flags >> 2) & 1}, {(flags >> 3) & 1}, {(flags >> 4) & 1}')
    good = 'software trip: V_HIGH' in text and 'below low limit' in text and 'clear:' in text
    L(f'{tag}.e', J(good), 'The fault description names the sensor, the reading, the bound and how to clear',
      'software trip: V_HIGH <value> below low limit; clear: ...', text[:110] or '(empty)')
    us = r[21] / hz * 1e6
    start = 'enableSafetyApi()' if dev else 'setChannelWatch()'
    L(f'{tag}.f', J(0 < us <= SW_REACTION_US),
      f'Software check: from {start} to the outputs stopped (5 control periods of 100 us declared)',
      f'<= {SW_REACTION_US} us', f'{us:.0f} us')
    rc_ = r[24]
    L(f'{tag}.h', J((rc_ & 0xFF) == 3 and not (rc_ >> 8) & 1 and (rc_ >> 9) & 1 and r[25] & mask == 0 and s8(rc_ >> 16) == 0),
      'A software trip while the PWM is starting (the first output enable still arming the hardware trips): latched, '
      'no output left on, relay open; cleared afterwards', 'TRIP, outputs off, pins 0, relay open, clear 0',
      f'fault {rc_ & 0xFF}, output on {(rc_ >> 8) & 1}, pins {r[25] & mask:#05x}, relay open {(rc_ >> 9) & 1}, '
      f'clear {s8(rc_ >> 16)}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-flash', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    J, L = th.judge, th.line
    log = {'cases': [], 'results': {}, 'text': {}}
    res = {}
    for env in ENVS:
        r, text, entries = run_env(env, a.out, not a.no_flash)
        for e in entries:   # one flat case list for the review page, the build in each case name
            log['cases'].append(dict(e, case=f"{env} {e.get('case', '?')}", boots=e.get('boots', 0),
                                     status=e.get('status', '?')))
        if r is None:
            L('L0-API-00' if env == 'SAFETY_API' else 'L0-API-06', 'FAIL', f'{env} build, flash or protocol', 'all '
              'cases acknowledged', f'see build_{env}.txt and report.json')
            res[env] = None
            continue
        log['results'][env], log['text'][env] = r, text
        res[env] = (r, text)

    P = res.get('SAFETY_API')
    if P:
        r, text = P
        # L0-API-02 setChannelReaction -> leg safe states
        L('L0-API-02.a', J(r[1] & 0xFF == 0xAA), 'setChannelReaction(Open_Circuit): both outputs of LEG1 (timer A) and '
          'LEG2 (timer C) inactive on a fault', 'A 2/2, C 2/2', f'codes {r[1] & 0xFF:#04x}')
        L('L0-API-02.b', J(r[0] & 0xFFF == 0xA66 and (r[0] >> 16) & 1),
          'setChannelReaction(Short_Circuit) before the PWM: the HRTIM fault state of both legs turns the low side on '
          '(OUT2), timer D (no leg) stays off', 'A 2/1, C 2/1, D 2/2 (0xa66)',
          f'{r[0] & 0xFFF:#05x}, reaction Short {(r[0] >> 16) & 1}')
        L('L0-API-02.c', J((r[0] >> 17) & 1 and (r[0] >> 18) & 1),
          'Once the PWM is initialised a change is refused (the HRTIM fixes the fault state): reaction and fault state '
          'kept', 'kept, kept', f'{(r[0] >> 17) & 1}, {(r[0] >> 18) & 1}')
        # L0-API-03 tighten-only software thresholds
        w0, w1 = r[3], r[4]
        a_, b_, c_, d_ = (s8(w0 >> (8 * k)) for k in range(4))
        e_, f_, g_, h_ = (s8(w1 >> (8 * k)) for k in range(4))
        L('L0-API-03.a', J(a_ == -2 and f(r[5]) == 100.0 and b_ == 0 and f(r[6]) == 80.0),
          'setChannelThresholdMax(V_HIGH): 120 (looser than 100) refused, 80 accepted', 'rc -2 (max 100), rc 0 (80)',
          f'rc {a_} (max {f(r[5]):g}), rc {b_} ({f(r[6]):g})')
        L('L0-API-03.b', J(c_ == -2 and f(r[7]) == 70.0),
          'All or nothing: {V1_LOW 60, V2_LOW 75} with V2_LOW looser than 70 is refused and V1_LOW keeps 70',
          'rc -2, V1_LOW max 70', f'rc {c_}, V1_LOW max {f(r[7]):g}')
        L('L0-API-03.c', J(d_ == -2 and f(r[8]) == 10.0 and e_ == 0 and f(r[9]) == 20.0),
          'setChannelThresholdMin(V_HIGH): 5 (looser than 10) refused, 20 accepted', 'rc -2 (min 10), rc 0 (20)',
          f'rc {d_} (min {f(r[8]):g}), rc {e_} ({f(r[9]):g})')
        L('L0-API-03.d', J(f_ == -1 and h_ == -1 and g_ == 0),
          'Bad sensor (0) and bad count refused; a sensor with no board limit (TEMP_SENSOR_1) accepts any value',
          'rc -1, -1, 0', f'rc {f_}, {h_}, {g_}')
        # L0-API-04 NVS
        nv, nrc = r[10], r[14]
        r1, r2 = s8(nv >> 8), s8(nv >> 16)
        L('L0-API-04.a', J(r1 == -5 and f(r[11]) == 10.0),
          'NVS entry with V_HIGH min 5 (looser than 10): retrieveThreshold() ignores it, the devicetree value stays',
          'rc -5, min 10', f'rc {r1}, min {f(r[11]):g}')
        L('L0-API-04.b', J(r2 == 0 and f(r[12]) == 20.0 and f(r[13]) == 90.0),
          'NVS entry tighter than the devicetree (20..90): retrieved', 'rc 0, 20..90',
          f'rc {r2}, {f(r[12]):g}..{f(r[13]):g}')
        L('L0-API-04.c', J((nv >> 24) & 1 and s8(nrc) >= 0 and s8(nrc >> 8) >= 0 and s8(nrc >> 16) >= 0),
          "The board's own NVS entry is put back (or the test entry deleted) after the case", 'restored',
          f"existed before {nv & 1}, restored {(nv >> 24) & 1}, store rc {s8(nrc)}, {s8(nrc >> 8)}, restore rc {s8(nrc >> 16)}")
        # L0-API-05 software check like every other trip, disableSafetyApi() ignored in production
        sw = r[15]
        L('L0-API-05.g', J(((sw >> 8) & 0xFF) == 3 and not (sw >> 16) & 1),
          'Production: disableSafetyApi() is ignored, the software check trips anyway', 'TRIP within 50 ms, outputs off',
          f'fault {(sw >> 8) & 0xFF}, outputs on {(sw >> 16) & 1}')
        judge_common('L0-API-05', r, text, dev=False)
        L('L0-API-06.a', J(r[2] & 3 == 0 and r[23] == 0), 'Production build reports production: telemetry flag 0, '
          'isDevelopmentMode() false', 'flag 0, false', f'flag {r[2] & 1}, isDevelopmentMode {(r[2] >> 1) & 1}')

    D = res.get('SAFETY_API_DEV')
    if D:
        r, text = D
        dv = r[2]
        L('L0-API-06.b', J(dv & 3 == 3 and r[23] == 1), 'Development build reports it: telemetry flag 1, '
          'isDevelopmentMode() true', 'flag 1, true', f'flag {dv & 1}, isDevelopmentMode {(dv >> 1) & 1}')
        L('L0-API-06.c', J(s8(dv >> 8) == 0 and (dv >> 16) & 1 and not (dv >> 17) & 1 and (dv >> 18) & 1),
          'disarm() before the first PWM output: PWM runs with the hardware trips unarmed, CPU supervision armed',
          'rc 0, outputs on, trips not armed, supervision armed',
          f'rc {s8(dv >> 8)}, outputs {(dv >> 16) & 1}, trips armed {(dv >> 17) & 1}, supervision {(dv >> 18) & 1}')
        a_, c_, d_ = s8(r[3]), s8(r[3] >> 16), s8(r[3] >> 24)
        L('L0-API-06.d', J(a_ == 0 and f(r[5]) == 120.0 and c_ == 0 and f(r[7]) == 60.0 and d_ == 0 and f(r[8]) == 5.0),
          'Development: software thresholds may be loosened beyond the devicetree', 'max 120, V1_LOW 60, min 5 accepted',
          f'rc {a_} ({f(r[5]):g}), rc {c_} ({f(r[7]):g}), rc {d_} ({f(r[8]):g})')
        r1 = s8(r[10] >> 8)
        L('L0-API-06.e', J(r1 == 0 and f(r[11]) == 5.0), 'Development: a looser NVS entry is retrieved',
          'rc 0, min 5', f'rc {r1}, min {f(r[11]):g}')
        sw = r[15]
        L('L0-API-06.f', J(((sw >> 8) & 0xFF) == 0 and (sw >> 16) & 1),
          'Development: disableSafetyApi() holds the software check off (V_HIGH beyond its limit for 50 ms, PWM on)',
          'no fault, outputs on', f'fault {(sw >> 8) & 0xFF}, outputs on {(sw >> 16) & 1}')
        judge_common('L0-API-06.g', r, text, dev=True)

    with open(os.path.join(a.out, 'report.json'), 'w') as fh:
        json.dump(log, fh, indent=1)
    th.write_target_manifest(
        a.out, 'L0-API · One Safety API on the Spin',
        'Checks that the legacy Safety API functions are compatibility wrappers of the protection: setChannelReaction() '
        'sets the leg safe states, software thresholds and NVS values are tighten-only, the software check reacts '
        'like every other trip (latched, stopped through the HRTIM, readable text), and that disableSafetyApi(), '
        'disarm() and loosening work only in development mode (ADR 0009).',
        ['L0-API-'],
        [['Board', 'Bare Spin with the Twist 1.4.1 devicetree (V_HIGH 10..100, V1_LOW/V2_LOW -10..70); current low '
                   'limits lowered to -11 A for the floating pins; safe output TEST_RELAY on PB9'],
         ['Builds', 'env SAFETY_API (production) and SAFETY_API_DEV (the same sources and devicetree plus '
                    'safety-mode = "development"), flashed one after the other'],
         ['Software trip', 'V_HIGH reads about 0 V on a bare Spin; its software min is tightened to 99; control task '
                           '100 us with data dispatch, PWM 200 kHz on timers A, C, D, E, F'],
         ['NVS', "V_HIGH entry (0x0303) written by the test; the board's own entry, if any, is restored afterwards"],
         ['Build test', 'L0-API-01 (build_mode.py): disarm() and run-time self-tests off must not compile in '
                        'production']],
        'SAFETY_API + SAFETY_API_DEV', 'supervision')
    print('The board now runs the SAFETY_API_DEV test firmware: flash env USB before any bench session.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
