#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file test_allocation.py
## @brief L0-ALC-01..10: the protection allocation (ADR 0009, issue luizvilla/Core#17) on synthetic boards.
##
## Pure Python, no hardware, no build. Each criterion prints one result line (@@ format). Synthetic sensors use the
## Spin's real pins: PA1 (COMP1, ADC1/2 ch2), PC1 (COMP3), PA0 (COMP3 too), PC2/PC3 (no comparator).
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'zephyr', 'modules', 'owntech_safety_api',
                                'scripts'))
from safety_allocation import allocate  # noqa: E402


def line(cid, ok, what, expected, measured):
    print(f'@@\t{cid}\t{"PASS" if ok else "FAIL"}\t{what}\t{expected}\t{measured}', flush=True)


def sensor(pin, mcu, adcs=(('adc1', 2), ('adc2', 2)), gain=0.005, offset=-10.0, conv='LINEAR'):
    return {'gain': gain, 'offset': offset, 'conv': conv,
            'channels': [{'adc': a, 'channel': c, 'spin_pin': pin, 'mcu_pin': mcu} for a, c in adcs]}


def limit(name, high=7.0, low=-7.0, **kw):
    return dict({'sensor': name, 'high': high, 'low': low}, **kw)


def kinds(alloc, name):
    l = next(x for x in alloc['limits'] if x['sensor'] == name)
    return tuple((l['paths'][b] or {}).get('kind') for b in ('low', 'high'))


def main():
    # 01 comparator on the high bound, ADC-watchdog window on the low bound; counts from gain/offset
    a = allocate({'sensors': {'I1': sensor(30, 'PA1')}, 'limits': [limit('I1')]})
    l = a['limits'][0]
    hi, lo = l['paths']['high'], l['paths']['low']
    exp_counts = round((7.0 + 10.0) / 0.005)   # 3400
    line('L0-ALC-01.a', kinds(a, 'I1') == ('adc_watchdog', 'comparator'), 'Current on PA1: comparator high, watchdog low',
         "('adc_watchdog', 'comparator')", str(kinds(a, 'I1')))
    line('L0-ALC-01.b', hi['comp'] == 1 and hi['flt'] == 4 and lo['flt'] == 1, 'PA1: COMP1 -> FLT4, ADC1 AWD1 -> FLT1',
         'COMP1/FLT4, AWD FLT1', f"COMP{hi['comp']}/FLT{hi['flt']}, AWD FLT{lo['flt']}")
    line('L0-ALC-01.c', hi['dac_code'] == exp_counts and lo['counts_high'] == exp_counts,
         'Comparator DAC code and watchdog counts from the sensor gain and offset', str(exp_counts),
         f"DAC {hi['dac_code']}, AWD {lo['counts_high']}")

    # 02 COMP3 serves PA0 or PC1, never both
    a = allocate({'sensors': {'I2': sensor(25, 'PC1', (('adc1', 7),)), 'V1': sensor(29, 'PA0', (('adc1', 1),))},
                  'limits': [limit('I2'), limit('V1', 70, -10)]})
    c3 = [x['sensor'] for x in a['limits'] if (x['paths']['high'] or {}).get('comp') == 3]
    line('L0-ALC-02', c3 == ['I2'], 'COMP3 shared by PC1 and PA0: only the first (current) gets it', "['I2']", str(c3))

    # 03 FLT conflict: COMP1 takes FLT4, so ADC2 AWD1 (also FLT4) is not given out
    a = allocate({'sensors': {'I1': sensor(30, 'PA1', (('adc2', 2),)), 'IX': sensor(26, 'PC2', (('adc2', 8),))},
                  'limits': [limit('I1'), limit('IX')]})
    flts = [p['flt'] for x in a['limits'] for p in x['paths'].values() if p and 'flt' in p]
    awd = [(p['adc'], p['awd']) for x in a['limits'] for p in x['paths'].values() if p and p['kind'] == 'adc_watchdog']
    line('L0-ALC-03', ('adc2', 1) not in awd and a['resources']['flt'].get('FLT4', '').endswith('(COMP1)'),
         'COMP1 uses FLT4: ADC2 AWD1 (FLT4) is skipped', 'no ADC2 AWD1', f'watchdogs {sorted(set(awd))}')

    # 04 exhaustion: 5 non-comparator limits, 4 usable watchdogs (ADC1 AWD1-3, ADC2 AWD3; ADC2 AWD1/2 reserved for
    # peak-current mode) -> the 5th is an error; with an exception, covered
    sens = {f'V{i}': sensor(26, 'PC2', (('adc1', 8), ('adc2', 8))) for i in range(5)}
    lims = [limit(f'V{i}', 70, -10) for i in range(5)]
    a = allocate({'sensors': sens, 'limits': lims})
    line('L0-ALC-04.a', len(a['errors']) == 1 and 'V4' in a['errors'][0] and 'exception' in a['errors'][0],
         'Fifth limit with no free watchdog: one error naming it and the fix', '1 error on V4 with the fix',
         f"{len(a['errors'])} error(s): {a['errors'][0][:60] if a['errors'] else ''}")
    lims[4]['exception'] = 'test'
    a = allocate({'sensors': sens, 'limits': lims})
    line('L0-ALC-04.b', not a['errors'] and kinds(a, 'V4') == ('software', 'software'),
         'With an exception, the limit falls back to the software path', 'no error, software/software',
         f"{len(a['errors'])} errors, {kinds(a, 'V4')}")

    # 05 protection = software uses no hardware
    a = allocate({'sensors': {'I1': sensor(30, 'PA1')}, 'limits': [limit('I1', protection='software')]})
    line('L0-ALC-05', not a['resources']['flt'] and kinds(a, 'I1') == ('software', 'software'),
         'protection = software takes no comparator or fault input', 'no FLT used', str(a['resources']['flt']))

    # 06 priority decides who gets the last channel
    sens = {f'V{i}': sensor(26, 'PC2', (('adc1', 8), ('adc2', 8))) for i in range(5)}
    lims = [limit(f'V{i}', 70, -10, exception='x') for i in range(5)]
    lims[4]['priority'] = 1
    a = allocate({'sensors': sens, 'limits': lims})
    line('L0-ALC-06', kinds(a, 'V4')[1] == 'adc_watchdog' and kinds(a, 'V3')[1] == 'software',
         'priority 1 on the last-declared limit gives it a hardware window first', 'V4 watchdog, V3 software',
         f"V4 {kinds(a, 'V4')}, V3 {kinds(a, 'V3')}")

    # 07 comparator-bound = low: the comparator watches the low bound, inverted
    a = allocate({'sensors': {'I1': sensor(30, 'PA1')}, 'limits': [limit('I1', comparator_bound='low')]})
    lo = a['limits'][0]['paths']['low']
    line('L0-ALC-07', lo['kind'] == 'comparator' and lo['polarity'] == 'inverted',
         'comparator-bound low: comparator on the low bound, output inverted', 'comparator, inverted',
         f"{lo['kind']}, {lo.get('polarity')}")

    # 08 non-linear sensor: no hardware path (error without exception)
    a = allocate({'sensors': {'T': sensor(31, 'PB0', conv='LOG')}, 'limits': [limit('T', 90, -20)]})
    line('L0-ALC-08', len(a['errors']) == 1 and 'not a linear sensor' in a['errors'][0],
         'Non-linear (LOG) sensor: no comparator or watchdog threshold; error unless exception', '1 error, not linear',
         a['errors'][0][:70] if a['errors'] else 'no error')

    # 09 short-low leg needs a hardware trip on its own current
    base = {'sensors': {'I1': sensor(30, 'PA1')}, 'limits': [limit('I1')]}
    a = allocate(dict(base, legs=[{'name': 'LEG1', 'safe_state': 'short-low', 'current_sensor': 'I2'}]))
    b = allocate(dict(base, legs=[{'name': 'LEG1', 'safe_state': 'short-low', 'current_sensor': 'I1'}]))
    c = allocate(dict(base, legs=[{'name': 'LEG1', 'safe_state': 'short-low', 'current_sensor': None,
                                   'short_risk_accepted': 'motor brake, current limited by design'}]))
    line('L0-ALC-09', len(a['errors']) == 1 and not b['errors'] and not c['errors'],
         'short-low leg: error without a hardware trip on its own current; OK with one, or with a stated risk',
         'error, ok, ok', f"{len(a['errors'])}, {len(b['errors'])}, {len(c['errors'])}")

    # 10 safe output not declared safe at reset: an explicit warning
    a = allocate({'sensors': {}, 'limits': [], 'safe_outputs': [{'name': 'relay', 'spin_pin': 22, 'mcu_pin': 'PC13',
                                                                  'safe_level': 0, 'reset_state_safe': False}]})
    w = a['warnings'][0] if a['warnings'] else ''
    line('L0-ALC-10', 'Why it matters' in w and 'Fix:' in w and 'reset-state-safe' in w,
         'Safe output not declared safe at reset: warning explains why and how to fix', 'why + fix',
         w[:80] if w else 'no warning')
    # 13 the watchdog goes on the sensor's default ADC (the one data acquisition converts)
    sd = sensor(26, 'PC2', (('adc1', 8), ('adc2', 8)))
    sd['default_adc'] = 'adc2'
    a = allocate({'sensors': {'IH': sd}, 'limits': [limit('IH')]})
    p = a['limits'][0]['paths']['high']
    line('L0-ALC-13', p['kind'] == 'adc_watchdog' and p['adc'] == 'adc2',
         'A sensor converted on ADC2 by default gets an ADC2 watchdog, never ADC1', 'ADC2', p.get('adc', p['kind']))

    # 14 voltage: hardware on the high bound only by default; the window's low side is open
    a = allocate({'sensors': {'VH': sensor(27, 'PC3', (('adc1', 9),))}, 'limits': [limit('VH', 100, 10)]})
    l = a['limits'][0]
    line('L0-ALC-14', kinds(a, 'VH') == ('software', 'adc_watchdog') and l['paths']['high']['counts_low'] == 0,
         'A voltage limit needs hardware on its high bound only; the window low side is 0', "software low, watchdog high, low 0",
         f"{kinds(a, 'VH')}, low counts {l['paths']['high'].get('counts_low')}")

    # 15 ADC2 AWD1/AWD2 stay free for peak-current mode (EEV4/EEV5)
    sens = {f'V{i}': sensor(26, 'PC2', (('adc2', 8),)) for i in range(3)}
    a = allocate({'sensors': sens, 'limits': [limit(f'V{i}', 70, -10, exception='x') for i in range(3)]})
    used = set(a['resources']['adc_watchdogs'])
    line('L0-ALC-15', 'ADC2 AWD1' not in used and 'ADC2 AWD2' not in used,
         'ADC2 AWD1 and AWD2 are never assigned (EEV4/EEV5 belong to peak-current mode)', 'not used', str(sorted(used)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
