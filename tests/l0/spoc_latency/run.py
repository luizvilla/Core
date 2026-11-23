#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-LAT: trip latency and faults during dead time, with direct probes on the CUT (ADR 0009, issue
##        luizvilla/Core#23, C13 and the open part of C14).
##
## SAFETY_L0_TWIST on the CUT. The CUT's DAC holds I1 (PA1) at 0 A through CH1 -> TP1-TP3 -> CH2; the MUX injector
## (DAC source, through CH3) holds I2 (PC1) at 0 A and steps it across one of its thresholds. Scope channels:
##   CH1 direct probe (x10) on PA8, leg 1 high side;  CH2 direct probe (x10) on PA9, leg 1 low side;
##   CH3 SPOC BNC3 (the injector node TP5, for the record); CH4 direct probe (x10) on PC1, the comparator input.
## Cases:
##   01 the dead time of the test build is the devicetree's (read back from DTxR) and shows on the pins;
##   02 high bound (COMP -> FLT, 282 ns filter): from PC1 crossing the threshold to the last output going low, on
##      every trip whose output was high throughout, within comparator + filter + one filter sample;
##   03 low bound (ADC watchdog -> FLT, sampled once per PWM period): same, within one PWM period + the conversion;
##   04 every trip of 02 and 03: PA8 and PA9 never high together, both low after the trip and staying low;
##   05 trips that land in the dead time (both outputs low at the fault): the output due next never turns on.
## The trips are asynchronous to the PWM (the MUX is a separate MCU), so their phase spreads over the period.
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_latency/run.py --out DIR [--trips N]
import argparse
import importlib.util
import json
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path[:0] = [os.path.join(REPO, 'tests', 'common'), os.path.join(REPO, 'tests', 'common', 'spoc')]
_spec = importlib.util.spec_from_file_location('spoc_trip_run', os.path.join(REPO, 'tests', 'l0', 'spoc_trip', 'run.py'))
trip = importlib.util.module_from_spec(_spec)   # tests/l0/spoc_trip/run.py
_spec.loader.exec_module(trip)
from mux_controller import PIN_TO_ROUTE  # noqa: E402

ENV, SHIELD = 'SAFETY_L0_TWIST', 'twist:1_4_2'
CH_HI, CH_LO, CH_TP5, CH_PIN = 1, 2, 3, 4      # PA8, PA9, TP5 (SPOC BNC3), PC1
LOGIC_V = 1.65                                  # half of 3.3 V
LOW_V = 0.5                                     # an output "stays low" below this
FREQ_HZ = 200000
PERIOD_NS = 1e9 / FREQ_HZ
DEAD_NS = (100, 100)                            # Twist 1.4.2 default-dead-time (rise, fall)
FILTER_NS = 282                                 # HRTIM fault filter of the comparator paths (fHRTIM/8, N = 6)
FILTER_SAMPLE_NS = 47                           # one filter sample: the fault input is synchronised to it
COMP_MAX_NS = 31                                # DS12288 Table 60: comparator pin -> HRTIM output, max
HIGH_LIMIT_NS = COMP_MAX_NS + FILTER_NS + FILTER_SAMPLE_NS
ADC_CONV_NS = 1000                              # ADR 0009 (2026-11-23): watchdog path <= channels x period + one
                                                # conversion; the watched channel is converted every Nth trigger
STEP_HIGH_MV = 2000                             # I2 high threshold 1700 mV
STEP_LOW_MV = 100                               # I2 low threshold 296 mV
REPEAT_CLEAR_MS = 100                           # test build (app.conf)
DEAD_BAND_NS = 40                               # comparator path: turn-offs this far below the median are PWM edges
LOW_BAND_NS = 1e9                               # watchdog path: latency spreads over a period, no dead-time sorting


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def crossings(t, v, level, rising):
    """Times where v crosses level (linear interpolation)."""
    out = []
    for i in range(1, len(v)):
        a, b = v[i - 1], v[i]
        if (rising and a < level <= b) or (not rising and a >= level > b):
            out.append(t[i - 1] + (level - a) * (t[i] - t[i - 1]) / (b - a))
    return out


def at(t, v, when):
    """Sample of v nearest to time when."""
    i = min(range(len(t)), key=lambda k: abs(t[k] - when))
    return v[i]


class LatBench(trip.Bench):
    """L0-TRP's bench with its own routes: CH1 -> PA6 and CH2 -> PA1 (the CUT DAC holds I1), CH3 -> PC1."""

    def route_lat(self, pin_dac_target, pin_step, step_hold_mv):
        self.pwm(False)
        self.inj.level(int(round(step_hold_mv)))          # source first: a source change disables CH3
        self.mux.disable_all()
        sh = self.mux._require_shell()
        for ch, pin in (('ch1', trip.SOURCE_PIN), ('ch2', pin_dac_target), ('ch3', pin_step)):
            r = PIN_TO_ROUTE[pin]
            self.mux._set_mux_group(sh, self.mux._mux_path(r['mux'], ch), r['input_index'], True)

    reconnects = 0

    def reconnect_cut(self):
        """Reopens the CUT's ThingSet port after a USB drop (up to 60 s), then stops the PWM."""
        self.reconnects += 1
        try:
            self.ser.close()
        except Exception:
            pass
        t0, port = time.time(), None
        while time.time() - t0 < 60 and not port:
            port = trip.boards.find('cut', tries=1)
        if not port:
            raise RuntimeError('CUT ThingSet port gone for 60 s')
        self.ser = trip.serial.Serial(port, 115200, timeout=0.2, write_timeout=1.0)
        self.cut = trip.ThingSetShell(self.ser)
        self.cut.enter_thingset()
        self.pwm(False)
        time.sleep(REPEAT_CLEAR_MS / 1000 + 0.1)

    def setup_probes(self):
        # Direct x10 probes on CH1, CH2, CH4; CH3 is SPOC BNC3 behind its x10 divider. 0.5 V/div around 1.5 V.
        for ch in (1, 2, 3, 4):
            self.scope.setup_channel(ch, scale=0.5, offset=-1.5, probe=10)

    def capture(self, level_mv, thr_v, rising, scale_s, offset_s):
        """Arms a single capture on CH4 crossing thr_v, steps the injector to level_mv, returns the waveforms."""
        sc = self.scope
        sc.set_timebase(scale_s, offset_s)
        for cmd in (':TRIGger:MODE EDGE', ':TRIGger:COUPling DC', f':TRIGger:EDGE:SOURce CHANnel{CH_PIN}',
                    f':TRIGger:EDGE:SLOPe {"POSitive" if rising else "NEGative"}',
                    f':TRIGger:EDGE:LEVel {thr_v:.3f}', ':TRIGger:SWEep NORMal', ':SINGle'):
            sc.write(cmd)
        time.sleep(0.5)
        self.inj.level(int(level_mv))
        t0 = time.time()
        while time.time() - t0 < 5 and not sc.query(':TRIGger:STATus?').strip().upper().startswith('STOP'):
            time.sleep(0.05)
        stopped = sc.query(':TRIGger:STATus?').strip().upper().startswith('STOP')
        return stopped, {ch: sc.waveform(ch) for ch in (CH_HI, CH_LO, CH_PIN)}


def analyse(w, thr_v, rising):
    """One trip: crossing time, outputs at it, final turn-off after it, overlap, outputs after the turn-off.

    The turn-off is the last moment either output is above 0.5 V after the crossing (an output that starts to turn on
    as a dead time ends and is cut within one HRTIM clock never reaches 1.65 V, but counts). Normally the fault forces it, and
    turn-off minus crossing is the latency. When the fault lands in a dead time, the turn-off is the PWM's own edge
    before it (the output due next never turns on), and comes earlier than the latency: classify() sorts these out.
    """
    tp, vp = w[CH_PIN]
    th, vh = w[CH_HI]
    tl, vl = w[CH_LO]
    xs = crossings(tp, vp, thr_v, rising)
    if not xs:
        return {'error': 'no threshold crossing on PC1'}
    tx = xs[0]
    overlap = sum(1 for a, b in zip(vh, vl) if a > LOGIC_V and b > LOGIC_V)
    last = [t for t, a in zip(th, vh) if t >= tx and a > LOW_V] + [t for t, b in zip(tl, vl) if t >= tx and b > LOW_V]
    if not last:
        return {'error': 'both outputs low from the crossing on (PWM not running?)'}
    t_off = max(last)
    # the falling edge's tail: 30 ns, or three samples at a slow timebase (2 us/div: 20 ns per screen point)
    guard = max(30e-9, 3 * (th[1] - th[0]))
    peak_after = max([v for t, v in zip(th, vh) if t > t_off + guard] + [v for t, v in zip(tl, vl) if t > t_off + guard]
                     or [0.0])
    return {'t_cross_ns': round(tx * 1e9, 2), 'hi_at': at(th, vh, tx) > LOGIC_V, 'lo_at': at(tl, vl, tx) > LOGIC_V,
            'overlap_samples': overlap, 'off_ns': round((t_off - tx) * 1e9, 1), 'peak_after_V': round(peak_after, 3),
            'record_after_off_ns': round((th[-1] - t_off) * 1e9)}


def classify(rows, band_ns):
    """Turn-offs within band_ns below the median (or any later) are latencies; earlier ones are dead-time trips."""
    offs = [r['off_ns'] for r in rows if not r.get('error')]
    if not offs:
        return None
    m = statistics.median(offs)
    for r in rows:
        if not r.get('error'):
            r['dead'] = r['off_ns'] < m - band_ns
            r['latency_ns'] = None if r['dead'] else r['off_ns']
    return m


def trips(b, n, zero1, hold2, level_mv, thr_v, rising, scale_s, offset_s, flt):
    out, i = [], 0
    while i < n:
        try:
            a = one_trip(b, zero1, hold2, level_mv, thr_v, rising, scale_s, offset_s, flt, keep_wave=i < 2)
        except (OSError, trip.serial.SerialException) as exc:
            # the CUT's USB link dropped (seen once in 132 trips, the firmware kept running): reconnect, stop the
            # PWM, repeat the trip
            print(f'  trip {i + 1}/{n}: CUT link lost ({exc}); reconnecting', flush=True)
            b.reconnect_cut()
            if b.reconnects > 5:
                out.append({'error': 'CUT link lost more than 5 times'})
                break
            continue
        out.append(a)
        i += 1
        if a.get('error', '').startswith('arming'):
            break
        print(f'  trip {i}/{n}: ' + (a.get('error') or f"off {a['off_ns']} ns after the crossing, "
              f"overlap {a['overlap_samples']}, peak after {a['peak_after_V']} V, {a['text']}"), flush=True)
    return out


def one_trip(b, zero1, hold2, level_mv, thr_v, rising, scale_s, offset_s, flt, keep_wave):
    ok, why, _ = b.arm(zero1, None)
    if not ok:
        return {'error': f'arming: {why}'}
    stopped, w = b.capture(level_mv, thr_v, rising, scale_s, offset_s)
    st = b.safety()
    a = analyse(w, thr_v, rising) if stopped else {'error': 'no trigger'}
    a.update({'fault': st['rFault'], 'tripped': st['rTripped'], 'text': st['rText'],
              'flt_ok': bool(st['rFault']) and bool(st['rTripped'] & (1 << (flt - 1)))})
    odd = not a.get('error') and (a['peak_after_V'] >= 0.2 or a['overlap_samples'] or a['record_after_off_ns'] < 200
                                  or (rising and a['off_ns'] < 250))
    if (keep_wave or odd) and stopped:   # the first two waveforms and every odd one, for the report (decimated)
        a['wave'] = {str(ch): [[round(t * 1e9 - a.get('t_cross_ns', 0), 1), round(v, 3)]
                               for t, v in list(zip(*w[ch]))[::2]] for ch in w}
    b.inj.level(int(round(hold2)))
    b.pwm(False)
    time.sleep(REPEAT_CLEAR_MS / 1000 + 0.1)
    b.cut.set_value('Safety/xClear', 'true')
    time.sleep(0.15)
    return a


def summary(rows):
    lats = [r['latency_ns'] for r in rows if r.get('latency_ns') is not None]
    return {'n': len(rows), 'clean': len(lats), 'min': min(lats) if lats else None,
            'median': round(statistics.median(lats), 1) if lats else None, 'max': max(lats) if lats else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-flash', action='store_true')
    ap.add_argument('--trips', type=int, default=300, help='high-bound trips (latency and dead time)')
    ap.add_argument('--low-trips', type=int, default=30, help='low-bound trips')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    trip.production_manifest(SHIELD, args.out)
    man = trip.build_and_flash(ENV, args.out, flash=not args.no_flash)
    held = trip.held_sensors(man)
    if len(held) != 2:
        line('L0-LAT-01', 'BLOCKED', 'Trip latency', 'two held current pins', f'held: {held}')
        return 0
    s1, s2 = held                                   # Twist: I1_LOW (PA1), I2_LOW (PC1)
    cases = trip.cases_from(man, 2048.0)
    hi = next(c for c in cases if c['sensor'] == s2 and c['bound'] == 'high' and c['kind'] == 'comparator')
    lo = next(c for c in cases if c['sensor'] == s2 and c['bound'] == 'low' and c['kind'] == 'adc_watchdog')
    p1 = next(c for c in cases if c['sensor'] == s1)['pin']
    seq = [n for n, sn in man['model']['sensors'].items() if sn.get('default_adc') == lo['path']['adc']]
    low_limit_ns = len(seq) * PERIOD_NS + ADC_CONV_NS
    rep = {'adc_sequence': {'adc': lo['path']['adc'], 'channels': seq, 'limit_ns': low_limit_ns},
           'high': {'case': {k: hi[k] for k in ('sensor', 'bound', 'kind', 'pin', 'threshold_mV')}},
           'low': {'case': {k: lo[k] for k in ('sensor', 'bound', 'kind', 'pin', 'threshold_mV')}}}

    trip.reset_cut()
    time.sleep(5)
    b = LatBench(man)
    try:
        b.vref = float(b.group('Inj')['rVref_mV'])
        b.setup_probes()
        zero1, zero2 = trip.zero_mv(man, s1, b.vref), trip.zero_mv(man, s2, b.vref)
        b.route_lat(p1, hi['pin'], zero2)
        ok, why, _ = b.arm(zero1, None)             # first arming: both pins at 0 A
        # probes in place: the PWM shows on CH1 and CH2 (direct probes), not through the SPOC dividers
        b.scope.set_timebase(2e-6, 0)
        b.scope.set_edge_trigger(CH_HI, LOGIC_V, sweep='AUTO')
        b.scope.run()
        time.sleep(1.0)
        seen = {ch: (b.scope.measure('FREQuency', ch), b.scope.measure('VMIN', ch), b.scope.measure('VMAX', ch))
                for ch in (CH_HI, CH_LO)}
        # a PWM pin swings 0 -> 3.3 V at 200 kHz; a SPOC node (BNC still on) sits near its held level with the
        # switching spikes, two per period, coupled onto it
        probes = all(f and abs(f - FREQ_HZ) < 0.05 * FREQ_HZ and lo is not None and hi is not None and lo < LOW_V
                     and hi > 2.5 for f, lo, hi in seen.values())
        if not ok or not probes:
            b.pwm(False)
            got = '; '.join(f'CH{ch} {f} Hz, {lo} to {hi} V' for ch, (f, lo, hi) in seen.items())
            for n in ('01', '02', '03', '04', '05'):
                line(f'L0-LAT-{n}', 'BLOCKED', 'Trip latency and dead time', 'direct probes CH1 PA8, CH2 PA9, CH4 PC1',
                     f'no {FREQ_HZ // 1000} kHz 0-3.3 V PWM on CH1/CH2 ({got})' + ('' if ok else f'; arming: {why}'))
            return 0

        # 01 dead time: register and pins
        pw = b.group('Pwm')
        dtr = int(pw.get('rDtR', 0))
        prsc = (dtr >> 10) & 0x7                      # DTxR: DTR[8:0] rising, DTPRSC[12:10], DTF[24:16] falling
        dtr_r, dtr_f = dtr & 0x1FF, (dtr >> 16) & 0x1FF
        sc = b.scope
        sc.set_timebase(50e-9, 0)
        sc.set_edge_trigger(CH_HI, LOGIC_V, slope='NEGative', sweep='NORMal')
        sc.write(':SINGle')
        time.sleep(1.0)
        th, vh = sc.waveform(CH_HI)
        tl, vl = sc.waveform(CH_LO)
        f_hi = [x for x in crossings(th, vh, LOGIC_V, False)]
        r_lo = [x for x in crossings(tl, vl, LOGIC_V, True)]
        dt_fall = ((min(x for x in r_lo if x > f_hi[0]) - f_hi[0]) * 1e9) if f_hi and any(x > f_hi[0] for x in r_lo) else None
        sc.set_edge_trigger(CH_LO, LOGIC_V, slope='NEGative', sweep='NORMal')
        sc.write(':SINGle')
        time.sleep(1.0)
        th, vh = sc.waveform(CH_HI)
        tl, vl = sc.waveform(CH_LO)
        f_lo = crossings(tl, vl, LOGIC_V, False)
        r_hi = crossings(th, vh, LOGIC_V, True)
        dt_rise = ((min(x for x in r_hi if x > f_lo[0]) - f_lo[0]) * 1e9) if f_lo and any(x > f_lo[0] for x in r_hi) else None
        b.pwm(False)
        rep['01'] = {'armed': ok, 'why': why, 'rDtR': dtr, 'dtr_counts': [dtr_r, dtr_f], 'dtprsc': prsc,
                     'pins_ns': {'PA8 fall -> PA9 rise': dt_fall, 'PA9 fall -> PA8 rise': dt_rise},
                     'udm': pw.get('rUdm'), 'per': pw.get('rPer')}
        dts = [d for d in (dt_fall, dt_rise) if d is not None]
        line('L0-LAT-01', judge(ok and len(dts) == 2 and all(abs(d - 100) <= 25 for d in dts)),
             f'Dead time of leg 1 on the pins (direct probes PA8/PA9, 50 ns/div), devicetree {DEAD_NS[0]}/{DEAD_NS[1]} ns',
             'both dead times 100 ns ±25 ns at 1.65 V (probe and driver edges included)',
             f"PA8 fall -> PA9 rise {dt_fall if dt_fall is None else round(dt_fall, 1)} ns, PA9 fall -> PA8 rise "
             f"{dt_rise if dt_rise is None else round(dt_rise, 1)} ns; DTxR 0x{dtr:08x}"
             + ('' if ok else f'; arming: {why}'))

        # 02 high bound, then 04/05 on the same trips
        thr_hi = hi['threshold_mV'] / 1000
        rows_hi = trips(b, args.trips, zero1, zero2, STEP_HIGH_MV, thr_hi, True, 100e-9, 350e-9,
                        hi['path']['flt'])
        # 03 low bound
        thr_lo = lo['threshold_mV'] / 1000
        rows_lo = trips(b, args.low_trips, zero1, zero2, STEP_LOW_MV, thr_lo, False, 2e-6, 8e-6,
                        lo['path']['flt'])
    finally:
        b.close()
    trip.reset_cut()

    rep['reconnects'] = b.reconnects
    rep['high']['median_off_ns'] = classify(rows_hi, DEAD_BAND_NS)
    rep['low']['median_off_ns'] = classify(rows_lo, LOW_BAND_NS)
    rep['high']['trips'], rep['low']['trips'] = rows_hi, rows_lo
    sh, sl = summary(rows_hi), summary(rows_lo)
    rep['high']['summary'], rep['low']['summary'] = sh, sl
    bad_hi = [r for r in rows_hi if r.get('error') or not r['flt_ok']]
    bad_lo = [r for r in rows_lo if r.get('error') or not r['flt_ok']]
    line('L0-LAT-02', judge(sh['clean'] >= 10 and not bad_hi and sh['max'] <= HIGH_LIMIT_NS),
         f"{s2} high bound (COMP{hi['path']['comp']} -> FLT{hi['path']['flt']}): PC1 stepped from its 0 A level to "
         f"{STEP_HIGH_MV} mV by the MUX injector; from PC1 crossing {hi['threshold_mV']:.0f} mV to the outputs below {LOW_V} V",
         f'<= {HIGH_LIMIT_NS} ns (comparator {COMP_MAX_NS} + filter {FILTER_NS} + one filter sample '
         f'{FILTER_SAMPLE_NS}) on >= 10 trips with the output high throughout; FLT{hi["path"]["flt"]} on every trip',
         f"{sh['clean']} of {sh['n']} trips measured: min {sh['min']}, median {sh['median']}, max {sh['max']} ns"
         + (f'; {len(bad_hi)} trips without FLT{hi["path"]["flt"]} or no capture' if bad_hi else '')
         + (f'; CUT USB link re-opened {b.reconnects} times, those trips repeated' if b.reconnects else ''))
    unended = [r for r in rows_lo if not r.get('error') and r['record_after_off_ns'] < 200]
    line('L0-LAT-03', judge(sl['clean'] >= 10 and not bad_lo and not unended and sl['max'] <= low_limit_ns),
         f"{s2} low bound ({lo['path']['adc'].upper()} AWD{lo['path']['awd']} -> FLT{lo['path']['flt']}): PC1 stepped "
         f"to {STEP_LOW_MV} mV; from PC1 crossing {lo['threshold_mV']:.0f} mV to the outputs below {LOW_V} V",
         f'<= {low_limit_ns:.0f} ns ({len(seq)} channels on {lo["path"]["adc"].upper()} ({", ".join(seq)}), one per '
         f'trigger: {len(seq)} x {PERIOD_NS:.0f} ns + one conversion {ADC_CONV_NS} ns; ADR 0009) on >= 10 trips, each '
         f'turned off inside the record; FLT{lo["path"]["flt"]} on every trip',
         f"{sl['clean']} of {sl['n']} trips measured: min {sl['min']}, median {sl['median']}, max {sl['max']} ns"
         + (f'; {len(bad_lo)} trips without FLT{lo["path"]["flt"]} or no capture' if bad_lo else '')
         + (f'; {len(unended)} still on at the end of the record' if unended else ''))
    allr = [r for r in rows_hi + rows_lo if not r.get('error')]
    overl = [r for r in allr if r['overlap_samples']]
    stay = [r for r in allr if r['peak_after_V'] >= LOW_V]
    line('L0-LAT-04', judge(allr and not overl and not stay),
         'Every trip of 02 and 03: PA8 and PA9 (leg 1, safe state open) on the direct probes',
         f'never both above {LOGIC_V} V; both below {LOW_V} V from 30 ns (or three samples) after the turn-off to the '
         'end of the record',
         f"{len(allr)} trips; both high together on {len(overl)}; an output above {LOW_V} V after the fault on "
         f"{len(stay)}" + (f" (peaks {sorted(r['peak_after_V'] for r in stay)[-3:]} V)" if stay else ''))
    dead = [r for r in rows_hi if not r.get('error') and r['dead']]
    dead_bad = [r for r in dead if r['peak_after_V'] >= LOW_V or r['overlap_samples']]
    rep['dead'] = {'n': len(dead), 'bad': len(dead_bad)}
    short = [r for r in dead if r['record_after_off_ns'] < rep['high']['median_off_ns'] - r['off_ns'] + 400]
    dead_bad += [r for r in short if r not in dead_bad]   # the record must run >= 400 ns past the fault
    line('L0-LAT-05', judge(len(dead) >= 3 and not dead_bad) if dead else 'INFO',
         f'High-bound trips whose fault lands in the {DEAD_NS[0]} ns dead time: the last output turned off by a PWM edge '
         f'more than {DEAD_BAND_NS} ns before the median latency, and nothing on after it',
         f'>= 3 such trips; on each, the output due after the dead time never turns on (below {LOW_V} V to the end of '
         'the record, >= 400 ns after the fault)',
         f"{len(dead)} of {len(rows_hi)} trips in the dead time, {len(dead_bad)} with an output on after it"
         + (f"; PWM edge {min(r['off_ns'] for r in dead):.0f} to {max(r['off_ns'] for r in dead):.0f} ns after the "
            f"crossing (median latency {rep['high']['median_off_ns']:.0f} ns)" if dead else ''))

    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump(rep, fh)
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    man_out = {
        'title': 'L0-LAT · trip latency and faults during dead time (direct probes)',
        'purpose': 'How fast the hardware trips turn the outputs off, measured on the CUT pins, and what a fault does '
                   'when it lands in the dead time (ADR 0009, C13 and C14).',
        'criteria': ['L0-LAT-'],
        'conditions': [
            ['Firmware', f'{ENV}: production allocation of {SHIELD} plus the reserved injection DAC; legs as '
                         f'shield.power sets them up (up-down, {FREQ_HZ // 1000} kHz, duty 0.5, dead time '
                         f'{DEAD_NS[0]}/{DEAD_NS[1]} ns)'],
            ['Stimulus', f'MUX injector, DAC source through OPAMP4 (6.5 V/us) and CH3, on {s2} (PC1): from its 0 A '
                         f'level to {STEP_HIGH_MV} mV (high bound) or {STEP_LOW_MV} mV (low bound); {s1} (PA1) held '
                         'at 0 A by the CUT DAC through CH1 -> TP1-TP3 -> CH2'],
            ['Probes', 'direct x10 passive probes: CH1 PA8, CH2 PA9, CH4 PC1; CH3 SPOC BNC3 (TP5). The probes are not '
                       'deskewed (same model; < 1 ns)'],
            ['Capture', 'single shot on CH4 crossing the threshold; 100 ns/div (high bound), 2 us/div (low bound); '
                        'logic level 1.65 V'],
            ['Trips', f'{args.trips} high-bound, {args.low_trips} low-bound; asynchronous to the PWM; latency '
                      'measured only when one output was high from the crossing to its turn-off']],
        'target': [['Bench', 'SPOC: CUT Spin + MUX Spin, ST-Link on the CUT; scope MHO984'],
                   ['MUX', f'Core-MUX ({trip.MUX_FW})'], ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']],
        'report': 'report.json', 'figures': 'spoc_latency'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man_out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
