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
FIELDS = ['magic', 'layout', 'boot_count', 'ack', 'busy', 'latched_at_boot', 'pending'] + [f'r{i}' for i in range(50)]
# LATCH_RESET reboots: it runs first so the results of the other cases (some beyond the 18 backup registers)
# are all from the final boot. TIGHTEN changes the I1_LOW threshold: it runs last.
# CLEAR_RULE leaves the board needing a reboot: it runs last.
CASES = [('CLEAR', 0), ('LATCH_RESET', 7), ('CONFIG', 1), ('AWD_CONFIG', 8), ('SAFE_CONFIG', 10), ('LOCKS', 2),
         ('TRIP_I1', 3), ('TRIP_I2', 4), ('AWD_TRIP', 9), ('DAC_TAMPER', 5), ('TIGHTEN', 6), ('CLEAR_RULE', 11),
         ('FROZEN', 12)]
COMPARATOR_REACTION_US = 0.5   # declared, ADR 0009 (filter 282 ns + comparator 31 ns max)
WATCHDOG_REACTION_US = 60.0    # declared, ADR 0009: 3 switching periods at 50 kHz
ALL_FLT = 0b111111             # Twist 1.4.1: comparators FLT4/FLT5, watchdogs FLT1/FLT2/FLT3/FLT6
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
    # Start from a fresh boot: a self-test fault latched by a previous run (FROZEN) is cleared only by a reboot.
    th.openocd('reset run', check=False)
    log = {'cases': []}
    rep = None
    before_frozen = None
    tw = []
    for name, code in CASES:
        if name == 'FROZEN':
            before_frozen = rep   # FROZEN reboots: the RAM-only results of the other cases are taken before it
            tw = th.read_words(th.symbol(ENV, 'g_safety_cmp_text')[0], 24) or []
        rep, entry = p.run(name, code)
        log['cases'].append(entry)
    r = [rep[f'r{i}'] for i in range(50)]
    if before_frozen is not None:
        r = [before_frozen[f'r{i}'] for i in range(48)] + r[48:50]
    log['results'] = r
    boots = {c['case']: c['boots'] for c in log['cases']}
    ok = {c['case']: c['status'] == 'acknowledged' for c in log['cases']}
    hz = r[20] or 170_000_000
    expect = r[44]   # pin levels the fault state gives (each leg's safe state, from OUTxR)
    mask = r[21] or 0xFF3
    J, L = th.judge, th.line

    # L0-CMP-01 arming and configuration
    armed, fmask, ntrips = r[0] & 1, (r[0] >> 8) & 0x3F, (r[0] >> 16) & 0xFF
    L('L0-CMP-01.a', J(armed and (fmask & 0b11000) == 0b11000 and ntrips == 2),
      'Comparator trips armed on FLT4 (I1_LOW) and FLT5 (I2_LOW)', 'armed, FLT4+FLT5, 2 trips',
      f'armed {armed}, mask {fmask:06b}, {ntrips} trips')
    for cid, csr, dac, name in (('b', r[1], r[3], 'I1_LOW'), ('c', r[2], r[4], 'I2_LOW')):
        e = EXPECT[name]
        good = (csr & 1) and (csr >> 31) and ((csr >> 8) & 1) == e['comp_csr_inpsel'] and ((csr >> 4) & 7) == e['inmsel'] \
            and not (csr >> 15) & 1 and dac == e['dac']
        L(f'L0-CMP-01.{cid}', J(good), f'{name}: comparator enabled and locked, inputs and polarity as allocated, DAC threshold',
          f"EN, LOCK, INPSEL {e['comp_csr_inpsel']}, INMSEL {e['inmsel']}, POL 0, DAC {e['dac']}",
          f'CSR {csr:#010x}, DAC {dac}')
    i1, i2, fltr = r[5], r[6], r[7]   # FLTINR as read after AWD_CONFIG (all trips armed)
    f4 = (i1 >> 24) & 0xFF   # FLT4 fields in FLTINR1 bits 24-31: E, P, SRC0, F[3:0], LCK
    f5 = i2 & 0xFF           # FLT5 fields in FLTINR2 bits 0-7
    def fld(b):
        return dict(E=b & 1, P=(b >> 1) & 1, SRC0=(b >> 2) & 1, F=(b >> 3) & 0xF, LCK=(b >> 7) & 1)
    good = all(fld(b) == dict(E=1, P=1, SRC0=1, F=8, LCK=1) for b in (f4, f5)) and not ((r[6] >> 16) & 0x18)
    L('L0-CMP-01.d', J(good), 'FLT4/FLT5: enabled, active high, internal source, 282 ns filter, locked',
      'E=1 P=1 SRC=01 F=1000 LCK=1', f'FLT4 {fld(f4)}, FLT5 {fld(f5)}')
    L('L0-CMP-01.e', J((fltr & ALL_FLT) == ALL_FLT and (fltr >> 31) & 1),
      'Every timer A-F listens to every armed fault input (FLT1-6), FLTLCK set', 'FLT1-6EN, FLTLCK on all six',
      f'AND of FLTxR {fltr:#010x}')
    L('L0-CMP-01.f', J((r[8] & 0x7F) == 0x5F), 'Fault interrupt enabled for every armed fault input, not the system fault',
      'IER 0x5f (FLT1-5 bits 0-4, FLT6 bit 6)', f'IER {r[8]:#06x}')

    # L0-CMP-02 locks hold against wild writes
    lk = r[9]
    L('L0-CMP-02', J(lk == 0x1F), 'Writes to the locked COMP1/COMP3 CSR, FLTINR1/2 and every FLTxR change nothing',
      'all 5 kept', f'kept {lk:05b} (b0 COMP1, b1 COMP3, b2 FLTINR1, b3 FLTINR2, b4 FLTxR)')

    # L0-CMP-03/04 trips
    for cid, name, fi, ci, pi, fl, pa in (('L0-CMP-03', 'I1_LOW (PA1, COMP1, FLT4)', 10, 11, 12, 4, 40),
                                          ('L0-CMP-04', 'I2_LOW (PC1, COMP3, FLT5)', 13, 14, 15, 5, 41)):
        f, cyc, pins, path = r[fi], r[ci], r[pi], r[pa]
        mode = f & 3
        L(f'{cid}.a', J(mode in (1, 2) and (f >> 8) & 1 and (f >> 16) & (1 << (fl - 1))),
          f'{name}: a step above the limit sets the HRTIM fault flag', 'flag set',
          f"mode {'output' if mode == 1 else 'charged analog' if mode == 2 else 'none'}, tripped {(f >> 16) & 0x3F:06b}")
        us, pus = cyc / hz * 1e6, path / hz * 1e6
        L(f'{cid}.b', J(mode and 0 < pus <= COMPARATOR_REACTION_US),
          f'{name}: comparator output to HRTIM fault flag (comparator, 282 ns filter, fault input)',
          f'<= {COMPARATOR_REACTION_US} us', f'{pus:.3f} us ({path} cycles)')
        L(f'{cid}.f', 'INFO', f'{name}: pin step to fault flag, including the GPIO write, the pad and the polling loop',
          'information', f'{us:.3f} us ({cyc} cycles)')
        L(f'{cid}.c', J(mode and pins & mask == expect & mask), f'{name}: every PWM pin at its leg\'s safe state after the trip',
          f'{expect & mask:#05x} on mask {mask:#05x}', f'{pins & mask:#05x}')
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

    # L0-AWD-01 watchdog path configuration (Twist 1.4.1, test overlay: current low limits -11 A)
    c1, c2, tr1, tr2, tr3, tr3b = r[24], r[25], r[26], r[27], r[28], r[29]
    a2, a3, b3, ee1, ee2, data = r[30], r[31], r[32], r[33], r[34], r[35]
    L('L0-AWD-01.a', J((c1 >> 12) & 1 and (c2 >> 12) & 1 and (c1 >> 10) & 3 and (c2 >> 10) & 3),
      'ADC1/ADC2: overrun mode overwrite, hardware trigger', 'OVRMOD 1, EXTEN != 0',
      f'ADC1 CFGR {c1:#010x}, ADC2 CFGR {c2:#010x}')
    L('L0-AWD-01.b', J((c1 >> 23) & 1 and (c1 >> 22) & 1 and ((c1 >> 26) & 0x1F) == 2 and tr1 == (3400 << 16) | (1 << 12)),
      'I1_LOW on ADC1 AWD1: channel 2, single channel, window 0-3400, 2-conversion filter',
      'AWD1EN AWD1SGL AWD1CH=2, TR1 0x0d481000', f'CFGR {c1:#010x}, TR1 {tr1:#010x}')
    L('L0-AWD-01.c', J(a2 == 1 << 9 and tr2 == 208 << 16 and a3 == 1 << 1 and tr3 == 225 << 16 and b3 == 1 << 7 and tr3b == 212 << 16),
      'V_HIGH ADC1 AWD2 (ch 9, high 208), V1_LOW ADC1 AWD3 (ch 1, high 225), I2_LOW ADC2 AWD3 (ch 7, 0-212); 8-bit windows, low side open for voltages',
      'as allocated', f'AWD2CR {a2:#x} TR2 {tr2:#x}, AWD3CR {a3:#x} TR3 {tr3:#x}, ADC2 AWD3CR {b3:#x} TR3 {tr3b:#x}')
    srcs = [(ee1 >> (6 * k)) & 3 for k in range(3)] + [ee2 & 3]
    L('L0-AWD-01.d', J(srcs == [3, 3, 3, 3]), 'External events 1, 2, 3 and 6 sourced by the ADC watchdogs',
      'EE1/2/3/6 SRC = 11', f'EECR1 {ee1:#010x}, EECR2 {ee2:#010x}')
    i1, i2 = r[5], r[6]
    bytes_ = [(i1 >> (8 * k)) & 0xFF for k in range(3)] + [(i2 >> 8) & 0xFF]
    src1 = [(i2 >> (16 + k)) & 1 for k in (0, 1, 2, 5)]
    L('L0-AWD-01.e', J(all(b == 0x83 for b in bytes_) and src1 == [1, 1, 1, 1]),
      'FLT1, FLT2, FLT3, FLT6: enabled, active high, source external event, no filter, locked',
      'E=1 P=1 SRC=10 F=0 LCK=1', f'FLT1-3,6 {[hex(b) for b in bytes_]}, SRC1 {src1}')
    L('L0-AWD-01.f', J(data & 1 and ((data >> 8) & 0xFF) == 3 and ((data >> 16) & 0xFF) == 1 and (data >> 24) & 1),
      'Data acquisition started by the Safety API with the watched channels; every watchdog self-test passed',
      'started, ADC1 3 channels, ADC2 1, self-tests passed',
      f'started {data & 1}, ADC1 {(data >> 8) & 0xFF}, ADC2 {(data >> 16) & 0xFF}, passed {(data >> 24) & 1}')
    L('L0-AWD-01.g', J(r[39] == ALL_FLT), 'All six fault inputs armed (comparators and watchdogs)', f'{ALL_FLT:06b}',
      f'{r[39]:06b}')
    # L0-AWD-02 trip through a watchdog
    f, cyc, pins = r[36], r[37], r[38]
    mode = f & 3
    us = cyc / hz * 1e6
    L('L0-AWD-02.a', J(mode in (1, 2) and (f >> 8) & 1), 'V_HIGH above its window sets the HRTIM fault flag (FLT2)',
      'flag set', f"mode {'output' if mode == 1 else 'charged analog' if mode == 2 else 'none'}")
    L('L0-AWD-02.b', J(mode and 0 < us <= WATCHDOG_REACTION_US), 'V_HIGH: step to fault flag',
      f'<= {WATCHDOG_REACTION_US:g} us', f'{us:.2f} us ({cyc} cycles)')
    L('L0-AWD-02.c', J(mode and pins & mask == expect & mask), 'V_HIGH: every PWM pin at its leg\'s safe state after the trip',
      f'{expect & mask:#05x} on mask {mask:#05x}', f'{pins & mask:#05x}')
    L('L0-AWD-02.d', J((f >> 9) & 1 and (f >> 10) & 1 and (f >> 11) & 1 and (f >> 12) & 1 and (f >> 13) & 1),
      'V_HIGH: latched, PWM refused, clear refused while beyond, accepted once back, PWM back',
      'latched, refused, refused, cleared, back', f'flags {(f >> 9) & 0x1F:05b}')

    # L0-SST: safe states, safe outputs, fault record, clear rule (#20)
    oa, oc = r[42], r[43]
    def faults(outr):
        return ((outr >> 4) & 3, (outr >> 20) & 3)
    L('L0-SST-01.a', J(faults(oa) == (2, 2) and faults(oc) == (2, 1) and expect & mask == 0x020),
      'Fault levels from the devicetree: LEG1 (timer A) open, LEG2 (timer C) short-low: TC2 (low side) active',
      'A 10/10, C 10/01, expected pins 0x020', f'A {faults(oa)}, C {faults(oc)}, expected {expect & mask:#05x}')
    so, legs = r[45], r[47]
    rc_late = (legs >> 8) & 0xFF
    L('L0-SST-01.b', J((so >> 8) & 1 and (legs & 0xFF) == 2 and rc_late == 0xFF),
      'One safe output and two legs declared; setSafeState() refused once armed', 'outputs 1, legs 2, rc -1',
      f'outputs {(so >> 8) & 1}, legs {legs & 0xFF}, rc {rc_late - 256 if rc_late > 127 else rc_late}')
    L('L0-SST-02', J(r[12] & mask == 0x020 and r[38] & mask == 0x020),
      'After a comparator trip and a watchdog trip, LEG2 keeps its low side on and every other switch is off',
      '0x020 both', f'comparator {r[12] & mask:#05x}, watchdog {r[38] & mask:#05x}')
    L('L0-SST-03', J(so & 7 == 7), 'The safe output (test relay) drops to its safe level on a comparator trip, a '
      'watchdog trip and a threshold read-back', 'all three', f'{so & 7:03b} (b0 comparator, b1 watchdog, b2 read-back)')
    fp = (so >> 16) & 3
    L('L0-SST-04', J(r[17] & mask == 0x020 and fp == 1),
      'A fault found by software (threshold read-back) stops through a real hardware fault (external event forced): '
      'LEG2 shorted, not opened', '0x020, path external event', f"{r[17] & mask:#05x}, path {['none', 'external event', 'comparator'][fp]}")
    text = b''.join(w.to_bytes(4, 'little') for w in tw).split(b'\0')[0].decode(errors='replace')
    L('L0-SST-05', J('I1_LOW high limit (COMP1->FLT4)' in text), 'The fault description names the sensor, bound and path',
      'contains "I1_LOW high limit (COMP1->FLT4)"', text[:90] or '(empty)')
    cr = r[46]
    rcs = [((cr >> (8 * k)) & 0xFF) for k in range(3)]
    rcs = [x - 256 if x > 127 else x for x in rcs]
    L('L0-SST-06', J(rcs == [0, -2, -2]), 'A second fault within the repeat-clear window needs a reboot',
      'clear 0, then -2, still -2 later', str(rcs))

    # L0-STK-09 frozen current sensor (#21)
    fz = r[48]
    import struct as _st
    fv = _st.unpack('<f', _st.pack('<I', r[49]))[0]
    L('L0-STK-09', J((fz & 0xFF) == 4 and ((fz >> 8) & 0xFF) == 7 and (fz >> 24) & 1),
      'A current reading unchanged for 2000 control periods with the PWM on: frozen-sensor self-test fault latched',
      'fault 4 detail 7, a value read', f"fault {fz & 0xFF} detail {(fz >> 8) & 0xFF} sensor {(fz >> 16) & 0xFF}, "
      f"value read {(fz >> 24) & 1} ({fv:.3f})")

    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump(log, f, indent=1)
    th.write_target_manifest(
        a.out, 'L0-CMP / L0-AWD · Hardware trips on the Spin',
        'Checks that each comparator trip the build allocated is armed, self-tested and locked before the first PWM '
        'output, that a sensor beyond its limit turns every PWM output off in hardware within the declared time, that '
        'the locks hold, that a threshold changed behind the API trips, and that a trip stays latched (ADR 0009).',
        ['L0-CMP-', 'L0-AWD-', 'L0-SST-', 'L0-STK-09'],
        [['Board', 'Bare Spin with the Twist 1.4.1 devicetree: I1_LOW on PA1 (COMP1/DAC3_CH1 -> FLT4), I2_LOW on PC1 '
                   '(COMP3/DAC1_CH1 -> FLT5), high limit 7 A = DAC 3400; PWM A, C, D, E, F at 200 kHz'],
         ['Stimulus', 'comparator: the firmware steps the floating sensor pin to 3.3 V; watchdog: ADC1 stopped, '
                      'a window every reading falls out of, ADC1 restarted on its HRTIM trigger (a pin cannot hold a '
                      'level against the ADC sampling capacitor); test code only'],
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
