#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-CMP: the comparator path on the Spin through an ST-Link (ADR 0009, issue luizvilla/Core#18).
##
## Builds and flashes env SAFETY_COMP (tests/firmware/safety_comparator, Twist 1.4.1 devicetree on a bare Spin),
## runs every case over SWD, judges, and writes the test-case folder. No scope: a trip is provoked by stepping the
## sensor pin from the firmware; its time is the cycle count from the step to the HRTIM fault flag. It REFLASHES the
## board: flash env USB again before any bench session.
##
## @par Usage
##   python3 tests/l0/safety_comparator/run.py --out DIR [--no-flash]
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'common'))
import target_harness as th  # noqa: E402

ENV = 'SAFETY_COMP'
FIELDS = ['magic', 'layout', 'boot_count', 'ack', 'busy', 'latched_at_boot', 'pending'] + [f'r{i}' for i in range(24)]
# LATCH_RESET reboots: it runs first so the results of the other cases (some beyond the 18 backup registers)
# are all from the final boot. TIGHTEN changes the I1_LOW threshold: it runs last.
CASES = [('CLEAR', 0), ('LATCH_RESET', 7), ('CONFIG', 1), ('LOCKS', 2), ('TRIP_I1', 3), ('TRIP_I2', 4),
         ('DAC_TAMPER', 5), ('TIGHTEN', 6)]
COMPARATOR_REACTION_US = 0.5   # declared, ADR 0009 (filter 282 ns + comparator 31 ns max)
EXPECT = {'I1_LOW': dict(comp_csr_inpsel=0, inmsel=4, dac=3400, flt=4),   # Twist 1.4.1: COMP1/DAC3_CH1, PA1
          'I2_LOW': dict(comp_csr_inpsel=1, inmsel=5, dac=3400, flt=5)}   # COMP3/DAC1_CH1, PC1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-flash', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if not a.no_flash:
        print(f'== build and flash env {ENV}', flush=True)
        rc = th.flash(ENV, os.path.join(a.out, 'build.txt'))
        if rc != 0:
            th.line('L0-CMP-00', 'FAIL', 'SAFETY_COMP build or flash', 'exit 0', f'exit {rc}, see build.txt')
            return 1
    p = th.Protocol(ENV, 'g_safety_cmp_report', 'g_safety_cmp_cmd', FIELDS, 0x31504D43)
    log = {'cases': []}
    rep = None
    for name, code in CASES:
        rep, entry = p.run(name, code)
        log['cases'].append(entry)
    r = [rep[f'r{i}'] for i in range(24)]
    log['results'] = r
    boots = {c['case']: c['boots'] for c in log['cases']}
    ok = {c['case']: c['status'] == 'acknowledged' for c in log['cases']}
    hz = r[20] or 170_000_000
    mask = r[21] or 0xFF3
    J, L = th.judge, th.line

    # L0-CMP-01 arming and configuration
    armed, fmask, ntrips = r[0] & 1, (r[0] >> 8) & 0x3F, (r[0] >> 16) & 0xFF
    L('L0-CMP-01.a', J(armed and fmask == 0b11000 and ntrips == 2), 'Comparator trips armed on FLT4 (I1_LOW) and FLT5 (I2_LOW)',
      'armed, FLT4+FLT5, 2 trips', f'armed {armed}, mask {fmask:06b}, {ntrips} trips')
    for cid, csr, dac, name in (('b', r[1], r[3], 'I1_LOW'), ('c', r[2], r[4], 'I2_LOW')):
        e = EXPECT[name]
        good = (csr & 1) and (csr >> 31) and ((csr >> 8) & 1) == e['comp_csr_inpsel'] and ((csr >> 4) & 7) == e['inmsel'] \
            and not (csr >> 15) & 1 and dac == e['dac']
        L(f'L0-CMP-01.{cid}', J(good), f'{name}: comparator enabled and locked, inputs and polarity as allocated, DAC threshold',
          f"EN, LOCK, INPSEL {e['comp_csr_inpsel']}, INMSEL {e['inmsel']}, POL 0, DAC {e['dac']}",
          f'CSR {csr:#010x}, DAC {dac}')
    i1, i2, fltr = r[5], r[6], r[7]
    f4 = (i1 >> 24) & 0xFF   # FLT4 fields in FLTINR1 bits 24-31: E, P, SRC0, F[3:0], LCK
    f5 = i2 & 0xFF           # FLT5 fields in FLTINR2 bits 0-7
    def fld(b):
        return dict(E=b & 1, P=(b >> 1) & 1, SRC0=(b >> 2) & 1, F=(b >> 3) & 0xF, LCK=(b >> 7) & 1)
    good = all(fld(b) == dict(E=1, P=1, SRC0=1, F=8, LCK=1) for b in (f4, f5)) and not ((i2 >> 16) & 0x18)
    L('L0-CMP-01.d', J(good), 'FLT4/FLT5: enabled, active high, internal source, 282 ns filter, locked',
      'E=1 P=1 SRC=01 F=1000 LCK=1', f'FLT4 {fld(f4)}, FLT5 {fld(f5)}')
    L('L0-CMP-01.e', J((fltr & 0x18) == 0x18 and (fltr >> 31) & 1), 'Every timer A-F listens to FLT4 and FLT5, FLTLCK set',
      'FLT4EN FLT5EN FLTLCK on all six', f'AND of FLTxR {fltr:#010x}')
    L('L0-CMP-01.f', J((r[8] & 0x18) == 0x18), 'Fault interrupt enabled for FLT4/FLT5', 'IER FLT4IE FLT5IE', f'IER {r[8]:#06x}')

    # L0-CMP-02 locks hold against wild writes
    lk = r[9]
    L('L0-CMP-02', J(lk == 0x1F), 'Writes to the locked COMP1/COMP3 CSR, FLTINR1/2 and every FLTxR change nothing',
      'all 5 kept', f'kept {lk:05b} (b0 COMP1, b1 COMP3, b2 FLTINR1, b3 FLTINR2, b4 FLTxR)')

    # L0-CMP-03/04 trips
    for cid, name, fi, ci, pi, fl in (('L0-CMP-03', 'I1_LOW (PA1, COMP1, FLT4)', 10, 11, 12, 4),
                                      ('L0-CMP-04', 'I2_LOW (PC1, COMP3, FLT5)', 13, 14, 15, 5)):
        f, cyc, pins = r[fi], r[ci], r[pi]
        mode = f & 3
        L(f'{cid}.a', J(mode in (1, 2) and (f >> 8) & 1 and (f >> 16) & (1 << (fl - 1))),
          f'{name}: a step above the limit sets the HRTIM fault flag', 'flag set',
          f"mode {'output' if mode == 1 else 'charged analog' if mode == 2 else 'none'}, tripped {(f >> 16) & 0x3F:06b}")
        us = cyc / hz * 1e6
        L(f'{cid}.b', J(mode and 0 < us <= COMPARATOR_REACTION_US), f'{name}: step to fault flag',
          f'<= {COMPARATOR_REACTION_US} us', f'{us:.3f} us ({cyc} cycles)')
        L(f'{cid}.c', J(mode and pins & mask == 0), f'{name}: every switching PWM pin low after the trip',
          f'0x000 on mask {mask:#05x}', f'{pins & mask:#05x}')
        L(f'{cid}.d', J((f >> 9) & 1 and (f >> 10) & 1), f'{name}: trip latched, PWM refused while latched', 'latched, refused',
          f'latched {(f >> 9) & 1}, refused {(f >> 10) & 1}')
        L(f'{cid}.e', J((f >> 11) & 1 and (f >> 12) & 1 and (f >> 13) & 1),
          f'{name}: clear refused while beyond the limit, accepted once back, PWM back', 'refused, accepted, back',
          f'{(f >> 11) & 1}, {(f >> 12) & 1}, {(f >> 13) & 1}')

    # L0-CMP-05 threshold read-back
    t = r[16]
    L('L0-CMP-05.a', J(t & 1 and t & 2 and t & 4),
      'A threshold written behind the API (DAC3 DHR) trips within one control period: latched, outputs off',
      'latched TRIP, detail 0x100, outputs off', f'flags {t & 7:03b}, detail {r[22]:#x}')
    L('L0-CMP-05.b', J(t & 8), 'Clear accepted once the threshold is restored', 'cleared', str((t >> 3) & 1))

    # L0-CMP-06 tighten only
    tg = r[18]
    rc1, rc2, code = tg & 0xFF, (tg >> 8) & 0xFF, (tg >> 16) & 0xFFF
    L('L0-CMP-06', J(rc1 == 0 and rc2 == 0xFF and code == 3000),
      'setTripThreshold(I1_LOW, -7, 5) tightens (DAC 3000); then 6 A (looser) is refused', 'rc 0, rc -1, DAC 3000',
      f'rc {rc1 if rc1 < 128 else rc1 - 256}, rc {rc2 if rc2 < 128 else rc2 - 256}, DAC {code}')

    # L0-CMP-07 latch across reset
    lt = r[19]
    L('L0-CMP-07', J(boots['LATCH_RESET'] >= 1 and lt == 0xF),
      'A trip stays latched across a software reset; PWM refused until clearFault(), then back',
      'latched, refused, cleared, back', f"{boots['LATCH_RESET']} reset(s), flags {lt:04b}")

    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump(log, f, indent=1)
    th.write_target_manifest(
        a.out, 'L0-CMP · Comparator path on the Spin',
        'Checks that each comparator trip the build allocated is armed, self-tested and locked before the first PWM '
        'output, that a sensor beyond its limit turns every PWM output off in hardware within the declared time, that '
        'the locks hold, that a threshold changed behind the API trips, and that a trip stays latched (ADR 0009).',
        ['L0-CMP-'],
        [['Board', 'Bare Spin with the Twist 1.4.1 devicetree: I1_LOW on PA1 (COMP1/DAC3_CH1 -> FLT4), I2_LOW on PC1 '
                   '(COMP3/DAC1_CH1 -> FLT5), high limit 7 A = DAC 3400; PWM A, C, D, E, F at 200 kHz'],
         ['Stimulus', 'the firmware steps the floating sensor pin to 3.3 V (GPIO output, or charged then analog); '
                      'test code only'],
         ['Timing', 'DWT cycles from the step to the HRTIM fault flag; declared <= 0.5 us (282 ns filter + 31 ns '
                    'comparator). The pin itself rises in a few ns; a scope check comes with the SPOC bench'],
         ['Not covered here', 'the low-bound (inverted) comparator in service: the arming self-test drives every '
                              'comparator through the inverted polarity, the in-service case needs a stimulus that '
                              'holds a level (SPOC bench)']],
        ENV, 'supervision')
    print('The board now runs the SAFETY_COMP test firmware: flash env USB before any bench session.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
