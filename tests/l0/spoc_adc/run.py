#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-ADC: offset of the CUT's ADC at the low end of its range (issue luizvilla/Core#23, finding of
##        2026-11-20 in docs/safety/spoc_bench.md: the ADC read ~14 mV low near 300 mV on the bench).
##
## Separates the two candidate causes: the bench's source impedance against the sampling time, or the ADC itself.
## The CUT's DAC (DAC2_CH1, PA6) and its ADCs share the VREFBUF, and an ideal pair reads back its own code
## (DAC: VREF x code / 4096; ADC: Vin / VREF x 4096): the error is ADC counts - DAC code, whatever VREF's true value.
##   01 PA6 itself on ADC2 IN3: the DAC pin straight, no SPOC path. The ADC's own offset and gain.
##   02 the first current pin (Twist I1, PA1) through CH1 -> TP1-TP3 -> CH2: the bench path, on each ADC that reads
##      it. Against 01 at the same sampling time: what the path adds.
##   01-02 at each sampling time (SMPR codes 0-7, 2.5-640.5 cycles; production 12.5): a source impedance shows as an
##      error that shrinks with longer sampling.
##   03 the same pin converted just after VREFINT (1.21 V on the sampling capacitor): memory of the previous channel.
##   04 the production reading (regular group, DMA, DataAPI::peekAdcRaw()) armed with the PWM on, as the trips see it.
## Fresh boot for 01-03 (the probe needs data acquisition not yet started: /AdcJ in tests/firmware/safety_l0), then
## the first arming for 04. Scope CH1 (DAC node) and CH2 (pin node) recorded at each level, information only.
## Characterisation: INFO lines, no verdict.
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_adc/run.py --out DIR
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path[:0] = [os.path.join(REPO, 'tests', 'common'), os.path.join(REPO, 'tests', 'common', 'spoc')]
_spec = importlib.util.spec_from_file_location('spoc_trip_run', os.path.join(REPO, 'tests', 'l0', 'spoc_trip', 'run.py'))
trip = importlib.util.module_from_spec(_spec)   # tests/l0/spoc_trip/run.py
_spec.loader.exec_module(trip)

ENV, SHIELD = 'SAFETY_L0_TWIST', 'twist:1_4_2'
LEVELS_MV = [100, 150, 200, 300, 500, 1000, 1500, 1900]
SMP_CYCLES = [2.5, 6.5, 12.5, 24.5, 47.5, 92.5, 247.5, 640.5]   # SMPR codes 0-7 (RM0440), ADC clock 42.5 MHz
PROD_SMP = 2                                                    # 12.5 cycles: adc_core.c
N = 256
PROD_SENSOR = 'V_HIGH'                                          # 04: high watchdog only
STAT_S = 1.0                                                    # 04: statistics per state (10 000 task periods)
SWEEP_MV = [100, 1000]                                          # 05
SWEEP_TRIG = [0.02, 0.06, 0.1, 0.2, 0.3, 0.4, 0.45, 0.5, 0.55, 0.6, 0.7, 0.8, 0.9, 0.94, 0.98]
INJ_MV = [100, 200, 300, 1000]                                  # 06
DAC_PIN = ('PA6', 2, 3)                                         # DAC2_CH1 output = ADC2_IN3
VREFINT_CH = 18                                                 # ADC1


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def dac_code(mv, vref):
    """The code /Inj writes (main.cpp inj_apply_now)."""
    return 4095 if mv >= vref else (int(mv) * 4095 + int(vref) // 2) // int(vref)


def probe(b, adc, ch, smp, pre=0, n=N):
    for k, v in (('wAdc', adc), ('wChannel', ch), ('wSmp', smp), ('wPre', pre), ('wN', n)):
        ok, resp = b.cut.set_value(f'AdcJ/{k}', str(v))
        if not ok:
            raise RuntimeError(f'ThingSet write AdcJ/{k}={v} failed: {resp!r}')
    b.cut.set_value('AdcJ/xRead', 'true')
    r = b.group('AdcJ')
    if r['rRc'] != 0:
        raise RuntimeError(f'/AdcJ rc {r["rRc"]} (ADC{adc} ch {ch}, smp {smp}, pre {pre})')
    return r['rMean_x16'] / 16.0, r['rMin'], r['rMax']


def fit(points):
    """Least squares error = a + b * code over (code, error) pairs; returns (offset counts at code 0, gain error)."""
    n = len(points)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] ** 2 for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    b = (n * sxy - sx * sy) / (n * sxx - sx ** 2)
    return (sy - b * sx) / n, b


def production_phase(man, rep, held, lim, hi_thr, shield, full):
    """04-06 on a fresh boot, the legs set up as shield.power does (shield) or as found until 2026-11-20
    (left-aligned, ADCs on the HRTIM master period)."""
    res = {'shield': shield}
    trip.reset_cut()
    time.sleep(5)
    b = trip.Bench(man)
    try:
        b.vref = rep['vref_mV']
        b.cut.set_value('Pwm/wShield', 'true' if shield else 'false')   # before the first PWM start
        armed, why = b.first_arming(held)
        pin = b.pin_of(PROD_SENSOR)
        adc_n, ch = 1, int(lim['paths']['high']['channel'])
        res['production'] = {'sensor': PROD_SENSOR, 'pin': pin, 'adc': adc_n, 'channel': ch, 'armed': armed,
                             'why': why, 'high_mV': hi_thr, 'rows': []}
        if armed:
            b.route(pin, held)
            b.cut.set_value('Adc/wAdc', str(adc_n))
            b.cut.set_value('Adc/wChannel', str(ch))
            levels = [mv for mv in LEVELS_MV if mv < hi_thr - 40]
            ok, why2, _ = b.arm(levels[0], None)
            res['production']['why'] = why2
            b.pwm(True)
            pw = b.group('Pwm')
            b.pwm(False)
            res['production'].update(adc1r=pw['rAdc1R'], adc3r=pw['rAdc3R'], per=pw['rPer'], cmp3=pw['rCmp3'],
                                     udm=pw['rUdm'])
            for mv in levels:
                b.level(mv)
                code = dac_code(mv, b.vref)
                row = {'mV': mv, 'code': code}
                for state in ('on', 'off'):
                    b.pwm(state == 'on')
                    time.sleep(0.3)
                    b.cut.set_value('Adc/xStat', 'true')
                    time.sleep(STAT_S)
                    a = b.group('Adc')
                    mean = a['rMean_x16'] / 16.0
                    row[state] = {'n': a['rN'], 'mean': mean, 'min': a['rMin'], 'max': a['rMax'], 'err': mean - code,
                                  'err_min': a['rMin'] - code, 'err_max': a['rMax'] - code,
                                  'fault': b.safety()['rFault']}
                res['production']['rows'].append(row)
            b.pwm(False)
            # 05 the sample instant across the PWM period (HRTIM CMP3 of each leg), outputs on
            sweep = []
            for mv in SWEEP_MV:
                b.level(mv)
                code = dac_code(mv, b.vref)
                for trig in SWEEP_TRIG:
                    b.cut.set_value('Pwm/wTrig', f'{trig:.3f}')
                    b.pwm(True)
                    time.sleep(0.3)
                    b.cut.set_value('Adc/xStat', 'true')
                    time.sleep(STAT_S / 2)
                    a, pw = b.group('Adc'), b.group('Pwm')
                    mean = a['rMean_x16'] / 16.0
                    sweep.append({'mV': mv, 'code': code, 'trig': trig, 'cmp3': pw['rCmp3'], 'per': pw['rPer'],
                                  'adc1r': pw['rAdc1R'],
                                  'n': a['rN'], 'err': mean - code, 'err_min': a['rMin'] - code,
                                  'err_max': a['rMax'] - code, 'fault': b.safety()['rFault']})
            b.pwm(False)
            if shield:                                      # back to the shield's instant (enableAdcTrigger)
                b.cut.set_value('Pwm/wTrig', '0.06')
            res['trigger_sweep'] = sweep
            # 06 injected conversions while armed (SMPR kept), outputs on and off: the pin through the SPOC mux
            # against the DAC pin itself, at instants unrelated to the PWM
            inj = []
            for mv in INJ_MV:
                b.level(mv)
                code = dac_code(mv, b.vref)
                row = {'mV': mv, 'code': code}
                for state in ('on', 'off'):
                    b.pwm(state == 'on')
                    time.sleep(0.3)
                    for key, (adc, ch) in (('pin', (1, int(lim['paths']['high']['channel']))), ('dac', DAC_PIN[1:])):
                        mean, lo, hi = probe(b, adc, ch, 255, n=1024)
                        row[f'{key}_{state}'] = {'err': mean - code, 'err_min': lo - code, 'err_max': hi - code}
                inj.append(row)
            b.pwm(False)
            res['injected_armed'] = inj
            if not full:
                return res
            # the pin node (CH2) at the lowest level with the outputs on, two PWM periods
            b.level(SWEEP_MV[0])
            b.pwm(True)
            sc = b.scope
            sc.setup_channel(2, scale=0.02, offset=-SWEEP_MV[0] / 1000, probe=10)
            sc.set_timebase(1e-6)
            sc.set_edge_trigger(2, SWEEP_MV[0] / 1000 - 0.01, slope='NEGative', sweep='AUTO')
            sc.single()
            t, v = sc.waveform(2)
            res['node_wave'] = {'mV': SWEEP_MV[0], 't': t, 'v': v}
            sc.run()
            b.pwm(False)
    finally:
        b.close()
    return res



def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-flash', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    trip.production_manifest(SHIELD, args.out)
    man = trip.build_and_flash(ENV, args.out, flash=not args.no_flash)
    held = trip.held_sensors(man)                              # Twist: I1_LOW (PA1), I2_LOW (PC1)
    rep = {'levels_mV': LEVELS_MV, 'smp_cycles': SMP_CYCLES, 'prod_smp': PROD_SMP, 'n': N, 'points': {},
           'pre': {}, 'scope': {}, 'production': {}}

    # 01-03 on a fresh boot, before any PWM start
    trip.reset_cut()
    time.sleep(5)
    b = trip.Bench(man)
    try:
        b.vref = float(b.group('Inj')['rVref_mV'])
        rep['vref_mV'] = b.vref
        rep['vrefint_cal'] = b.group('AdcJ')['rVrefintCal']
        rep['vrefint'] = probe(b, 1, VREFINT_CH, 7)[0]          # 640.5 cycles: VREFINT needs >= 4 us
        targets = [(held[0], b.pin_of(held[0])), (PROD_SENSOR, b.pin_of(PROD_SENSOR))]
        for k, (sensor, pin) in enumerate(targets):
            b.route(pin, [])                                    # CH1 = PA6 (DAC), CH2 = the pin; injector off
            points = {} if k else {f'{DAC_PIN[0]} ADC{DAC_PIN[1]} IN{DAC_PIN[2]} (DAC pin)': (DAC_PIN[1], DAC_PIN[2])}
            for c in man['model']['sensors'][sensor]['channels']:
                if c['mcu_pin'] == pin:
                    points[f'{pin} ADC{str(c["adc"])[-1]} IN{c["channel"]} ({sensor}, bench path)'] = \
                        (int(str(c['adc'])[-1]), int(c['channel']))
            for name in points:
                rep['points'][name] = {'adc_ch': points[name], 'pin': pin, 'rows': []}
            for mv in LEVELS_MV:
                b.level(mv)
                time.sleep(0.3)
                code = dac_code(mv, b.vref)
                v1, v2 = b.scope.measure('VAVG', 1), b.scope.measure('VAVG', 2)
                rep['scope'].setdefault(pin, {})[str(mv)] = {'code': code, 'ch1_V': v1, 'ch2_V': v2}
                for name, (adc, ch) in points.items():
                    for smp in range(8):
                        mean, lo, hi = probe(b, adc, ch, smp)
                        rep['points'][name]['rows'].append({'mV': mv, 'code': code, 'smp': smp, 'mean': mean,
                                                            'min': lo, 'max': hi, 'err': mean - code})
                    if adc != 1 or 'bench' not in name:
                        continue
                    for smp in (0, PROD_SMP, 7):
                        mean, lo, hi = probe(b, adc, ch, smp, pre=VREFINT_CH)
                        rep['pre'].setdefault(name, []).append({'mV': mv, 'code': code, 'smp': smp, 'mean': mean,
                                                                'min': lo, 'max': hi, 'err': mean - code})
        rep['vrefint_end'] = probe(b, 1, VREFINT_CH, 7)[0]
    finally:
        b.close()

    # 04 production path, armed: the regular group as the trips and the plausibility check see it, on a pin whose
    # only watchdog is high (V_HIGH), so it may go down to 100 mV; I1 and I2 held at 0 A on CH3 and CH4 (TP5-TP7).
    lim = next(l for l in man['limits'] if l['sensor'] == PROD_SENSOR)
    hi_thr = trip.threshold_mv(lim, 'high', lim['paths']['high'], rep['vref_mV'])
    rep.update(production_phase(man, rep, held, lim, hi_thr, shield=True, full=True))
    rep['as_found'] = production_phase(man, rep, held, lim, hi_thr, shield=False, full=False)

    # result lines
    lsb = rep['vref_mV'] / 4096
    vrefint_mv = 3000 * rep['vrefint_cal'] / rep['vrefint']
    line('L0-ADC-00', 'INFO', 'VREF+ from VREFINT (ADC1 IN18, 640.5 cycles; VREF+ = 3.0 V x VREFINT_CAL / reading)',
         f'{rep["vref_mV"]:.0f} mV (VREFBUF)',
         f'{vrefint_mv:.1f} mV (cal {rep["vrefint_cal"]}, reading {rep["vrefint"]:.1f} at start, '
         f'{rep["vrefint_end"]:.1f} at end)')
    for i, (name, p) in enumerate(rep['points'].items()):
        cid = 'L0-ADC-01' if i == 0 else f'L0-ADC-02.{p["pin"]}.{p["adc_ch"][0]}'
        per_smp = []
        for smp in range(8):
            rows = [r for r in p['rows'] if r['smp'] == smp]
            off, gain = fit([(r['code'], r['err']) for r in rows])
            at = {r['mV']: r['err'] for r in rows}
            per_smp.append((smp, off, gain, at))
            p.setdefault('fit', {})[smp] = {'offset_counts': off, 'gain_error': gain}
        prod = per_smp[PROD_SMP]
        line(cid, 'INFO', f'{name}: ADC counts - DAC code at {SMP_CYCLES[PROD_SMP]} cycles (production)',
             'ideal 0 counts', '; '.join(f'{mv} mV {e:+.1f}' for mv, e in prod[3].items())
             + f' (counts; 1 count = {lsb:.2f} mV); fit: offset {prod[1]:+.1f} counts, gain {prod[2] * 100:+.2f} %')
        line(cid + '.smp', 'INFO', f'{name}: error at 300 mV and fitted offset against the sampling time',
             'constant if the source impedance does not matter',
             '; '.join(f'{SMP_CYCLES[s]} cyc: {at.get(300, float("nan")):+.1f} / {off:+.1f}' for s, off, _, at in per_smp))
    for name, rows in rep['pre'].items():
        txt = []
        for smp in (0, PROD_SMP, 7):
            pre = {r['mV']: r['err'] for r in rows if r['smp'] == smp}
            plain = {r['mV']: r['err'] for r in rep['points'][name]['rows'] if r['smp'] == smp}
            txt.append(f'{SMP_CYCLES[smp]} cyc: ' + ', '.join(f'{mv} mV {pre[mv] - plain[mv]:+.1f}' for mv in (100, 300, 1900)))
        line(f'L0-ADC-03.{rep["points"][name]["pin"]}', 'INFO', f'{name}: converted right after VREFINT (1.21 V), change against alone (counts)',
             '0 if the sampling capacitor settles', '; '.join(txt))
    for res, sfx in ((rep, ''), (rep.get('as_found', {}), '.found')):
        pr = res.get('production', {})
        if pr.get('shield', res.get('shield', True)):
            how = (f'legs as shield.power sets them ({"up-down" if pr.get("udm") else "left-aligned"}, ADC1 from PWMA '
                   f'and ADC2 from PWMC CMP3 = {pr.get("cmp3")} / period {pr.get("per")})')
        else:
            how = f'legs as found until 2026-11-20 (left-aligned, HRTIM ADC1R 0x{pr.get("adc1r", 0):x}: master period)'
        if pr.get('rows'):
            for state in ('on', 'off'):
                line(f'L0-ADC-04{sfx}.{state}', 'INFO', f'Production reading of {pr["sensor"]} ({pr["pin"]}, ADC{pr["adc"]} '
                     f'IN{pr["channel"]}, regular group, newest sample each 100 us task period, {STAT_S:.0f} s), PWM '
                     f'outputs {state}, {how}: counts - DAC code, mean [min, max]', 'as 02 at 12.5 cycles',
                     '; '.join(f'{r["mV"]} mV {r[state]["err"]:+.1f} [{r[state]["err_min"]:+d}, {r[state]["err_max"]:+d}]'
                               for r in pr['rows']) + f' (n {pr["rows"][0][state]["n"]} per level)')
        else:
            line(f'L0-ADC-04{sfx}', 'INFO', 'Production reading armed', 'armed', f'not armed: {pr.get("why")}')
        for key, what in (('pin', f'{PROD_SENSOR} pin (PC3, ADC1 IN9) through the SPOC mux'),
                          ('dac', 'DAC pin (PA6, ADC2 IN3), no SPOC path')):
            rows = res.get('injected_armed', [])
            if rows:
                line(f'L0-ADC-06{sfx}.{key}', 'INFO', f'{what}: injected conversions while armed, {how}, PWM outputs '
                     'on / off: counts - DAC code, mean [min, max]', 'on = off if the switching does not reach this point',
                     '; '.join(f'{r["mV"]} mV on {r[key + "_on"]["err"]:+.1f} [{r[key + "_on"]["err_min"]:+d}, '
                               f'{r[key + "_on"]["err_max"]:+d}] / off {r[key + "_off"]["err"]:+.1f}' for r in rows))
        for mv in SWEEP_MV:
            rows = [r for r in res.get('trigger_sweep', []) if r['mV'] == mv]
            if rows:
                line(f'L0-ADC-05{sfx}.{mv}', 'INFO', f'V_HIGH at {mv} mV, PWM outputs on (duty 0.5), {how}: counts - '
                     f'DAC code against the CMP3 fraction (/Pwm wTrig; period register {rows[0]["per"]})',
                     'flat if the sample instant does not matter',
                     '; '.join(f'{r["trig"]:.2f} ({r["cmp3"]}): {r["err"]:+.1f} [{r["err_min"]:+d}, {r["err_max"]:+d}]'
                               for r in rows))

    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump(rep, fh)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    man_out = {
        'title': 'L0-ADC · offset of the CUT ADC at the low end (SPOC bench)',
        'purpose': 'Why the CUT ADC reads ~14 mV low near 300 mV (finding of 2026-11-20, #23): its own offset, or the '
                   'bench\'s source impedance against the 12.5-cycle sampling time. Characterisation, no verdict.',
        'criteria': ['L0-ADC-'],
        'conditions': [
            ['Firmware', f'{ENV}: /AdcJ injected conversions (software trigger, chosen SMPR, {N} per point), run '
                         'on a fresh boot before data acquisition; the ADC woken and calibrated as the ADC driver does'],
            ['Reference', 'the CUT DAC (DAC2_CH1, PA6) and the ADCs share the VREFBUF: error = ADC counts - DAC code'],
            ['Points', 'PA6 on ADC2 IN3 (no SPOC path); I1 (PA1) and V_HIGH (PC3) through CH1 -> TP1-TP3 -> CH2, on '
                       'each ADC reading them'],
            ['Levels', ', '.join(map(str, LEVELS_MV)) + ' mV'],
            ['Sampling', ', '.join(map(str, SMP_CYCLES)) + ' cycles at 42.5 MHz; production 12.5 (adc_core.c)'],
            ['04', f'first arming, then V_HIGH (PC3) from the DAC, I1 and I2 held at 0 A on CH3/CH4 (TP5-TP7); '
                   f'/Adc statistics {STAT_S:.0f} s per level with the PWM outputs on, then off (HRTIM still triggering)'],
            ['Legs', '04-06 run twice, each on a fresh boot: the legs as shield.power sets them from the devicetree '
                     '(/Pwm wShield true: up-down, ADC1 from PWMA and ADC2 from PWMC CMP3 at 0.06 x period), then as '
                     'found until 2026-11-20 (.found: left-aligned, ADCs on the HRTIM master period)'],
            ['05', 'the same, outputs on at duty 0.5, the ADC trigger instant (each leg\'s HRTIM CMP3, /Pwm wTrig) '
                   'swept over the period at ' + ' and '.join(map(str, SWEEP_MV)) + ' mV; then the node on CH2, '
                   '20 mV/div, 1 us/div'],
            ['06', 'armed, /AdcJ with SMPR kept (wSmp 255), 1024 injected conversions per point at instants unrelated '
                   'to the PWM: the V_HIGH pin through the SPOC mux and the DAC pin, outputs on then off']],
        'target': [['Bench', 'SPOC: CUT Spin + MUX Spin, ST-Link on the CUT'], ['MUX', f'Core-MUX ({trip.MUX_FW})'],
                   ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']],
        'report': 'report.json', 'figures': 'spoc_adc'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man_out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
