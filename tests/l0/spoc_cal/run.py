#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.py
## @brief L0-CAL: SPOC bench calibration at the start of a campaign (ADR 0009, issue luizvilla/Core#26).
##
## The CUT (Core-CUT) drives PA8 at 1 kHz, 0.5; the MUX (Core-MUX, branch mux-injector) routes it to the scope
## channels. Per channel, routed alone: compensation (from the waveform, the trim check), then the scope's own
## FREQuency, PDUTy, VTOP and VBASe (median of 5, with spread: the cable check). Per channel pair CH1 + CHn:
## the delay between 50 % crossings on stopped single captures at 20 ns/div (skew, information only: the scope's
## RRDelay/FFDelay were not usable on this bench). Gain and skew are recorded in
## calibration.json for the campaign. Prints one result line per criterion (@@ format) and writes the test-case
## folder (target_manifest.json, report.json, calibration.json).
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_cal/run.py --out DIR
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
import trim  # noqa: E402
from trim import rigol_mho, boards, waveform, MuxController  # noqa: E402

MUX_FW = 'owntech-foundation/SPOC_Software mux-injector fe04fe3'
CHANNELS = trim.CHANNELS
F_NOM, D_NOM, V_HIGH = 1000.0, 0.5, 3.3
F_TOL, D_TOL = 0.001, 0.005          # frequency ±0.1 %, duty ±0.005 (median and spread)
AMP_TOL, LEVEL_SPREAD = 0.05, 0.05   # amplitude ±5 % of 3.3 V; VTOP/VBASe spread <= 50 mV
N = 5


def line(cid, verdict, what, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{what}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def series_single(scope, item, ch, ch2=None, n=N, tries=2 * N):
    """Scope measurement on stopped single captures until n are valid (at most `tries` captures; some captures at
    20 ns/div hold no edge): (median, min, max, valid, tries used), or None with fewer than 3 valid."""
    vals, used = [], 0
    while len(vals) < n and used < tries:
        used += 1
        scope.single()
        v = scope.measure(item, ch, ch2)
        if v is not None:
            vals.append(v)
    if len(vals) < 3:
        return None
    return statistics.median(vals), min(vals), max(vals), len(vals), used


def channel_case(scope, mux, ch):
    trim.route_alone(mux, ch)
    scope.set_timebase(200e-6)
    scope.set_edge_trigger(ch, 1.65, sweep='AUTO')
    scope.run()
    time.sleep(1.0)
    t, v = scope.waveform(ch)
    comp = waveform.compensation_pct(t, v)
    meas = {item: scope.measure_series(item, ch, n=N) for item in ('FREQuency', 'PDUTy', 'VTOP', 'VBASe')}
    return {'channel': ch, 'compensation_pct': comp, 'measure': meas, 'wave': {'t': t, 'v': v}}


def mid_crossing(t, v, rising):
    """Time where v crosses 50 % of its own swing (1st and 99th percentiles), linearly interpolated."""
    sv = sorted(v)
    lo, hi = sv[len(sv) // 100], sv[-1 - len(sv) // 100]
    mid = (lo + hi) / 2
    for i in range(1, len(v)):
        a, b = v[i - 1], v[i]
        if (rising and a < mid <= b) or (not rising and a > mid >= b):
            return t[i - 1] + (mid - a) / (b - a) * (t[i] - t[i - 1])
    return None


def skew_case(scope, mux, ch, n=N):
    """CH1 -> CHn delay at the 50 % crossing of each channel, from n stopped single captures per edge (20 ns/div).
    From the waveforms: the scope's RRDelay read invalid and FFDelay a constant -0.2 ns (one screen sample) here."""
    mux.route_pair('ch1', trim.CUT_PIN, f'ch{ch}', trim.CUT_PIN)  # route() would disable the other route
    time.sleep(1.0)
    scope.set_timebase(20e-9)
    out = {'channel': ch}
    for slope, key in (('POSitive', 'rising'), ('NEGative', 'falling')):
        scope.set_edge_trigger(1, 1.65, slope=slope, sweep='NORMal')
        scope.run()
        time.sleep(0.5)
        vals, used = [], 0
        while len(vals) < n and used < 2 * n:
            used += 1
            scope.single()
            t1, v1 = scope.waveform(1)
            tn, vn = scope.waveform(ch)
            a, b = mid_crossing(t1, v1, key == 'rising'), mid_crossing(tn, vn, key == 'rising')
            if a is not None and b is not None:
                vals.append(b - a)
                if key == 'rising' and 'edge' not in out:
                    out['edge'] = {'ch1': {'t': t1, 'v': v1}, f'ch{ch}': {'t': tn, 'v': vn}}
        out[key] = (statistics.median(vals), min(vals), max(vals), len(vals), used) if len(vals) >= 3 else None
    return out


def target_snapshot(idn):
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    return [['Bench', 'SPOC: CUT Spin + MUX Spin soldered on the board, 9 MΩ dividers trimmed, BNC to the scope'],
            ['CUT', f'Spin 1.2.0, USB 2fe3:01ff, Core-CUT (SPOC_Software Core-CUT), port {boards.find("cut")}'],
            ['MUX', f'Spin 1.2.0, USB 2fe3:0100, Core-MUX ({MUX_FW}), port {boards.find("mux")}'],
            ['Scope', idn],
            ['Host code', f'Core2 {sha}{"-dirty" if dirty else ""}']]


def fmt(s, unit='', scale=1.0, digits=4):
    if s is None:
        return 'invalid'
    if isinstance(s, dict):
        s = (s['median'], s['min'], s['max'])
    text = f'{s[0] * scale:.{digits}g} {unit} (min {s[1] * scale:.{digits}g}, max {s[2] * scale:.{digits}g}'
    if len(s) > 3:
        text += f'; {s[3]} valid of {s[4]} captures'
    return (text + ')').replace('  ', ' ')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    scope = rigol_mho.RigolScope(rigol_mho.open_transport('/dev/usbtmc0'))
    idn = scope.idn()
    mux = MuxController(boards.find('mux'))
    mux.connect()
    try:
        trim.setup(scope, mux, boards.find('cut'))
        line('L0-CAL-01', 'PASS', 'Bench answers: CUT, MUX and scope', 'all three', f'{idn}; CUT and MUX over ThingSet')
        chans = [channel_case(scope, mux, ch) for ch in CHANNELS]
        skews = [skew_case(scope, mux, ch) for ch in CHANNELS[1:]]
    finally:
        mux.disable_all()
        mux.close()
        scope.set_edge_trigger(1, 1.65, sweep='AUTO')
        scope.set_timebase(200e-6)
        scope.run()
        scope.close()

    letters = 'abcd'
    amp1 = None
    for i, c in enumerate(chans):
        ch, m, L = c['channel'], c['measure'], letters[i]
        comp = c['compensation_pct']
        line(f'L0-CAL-02.{L}', judge(comp is not None and trim.TRIMMED[0] <= comp <= trim.TRIMMED[1]),
             f'CH{ch} divider compensation (trim)', '95-105 %', 'invalid' if comp is None else f'{comp:.1f} %')
        f = m['FREQuency']
        ok = f is not None and abs(f['median'] - F_NOM) <= F_TOL * F_NOM and f['max'] - f['min'] <= F_TOL * F_NOM
        line(f'L0-CAL-03.{L}', judge(ok), f'CH{ch} scope FREQuency, median and spread of {N}',
             f'{F_NOM:g} Hz ±{F_TOL:.1%}, spread <= {F_TOL:.1%}', fmt(f, 'Hz'))
        d = m['PDUTy']
        ok = d is not None and abs(d['median'] - D_NOM) <= D_TOL and d['max'] - d['min'] <= D_TOL
        line(f'L0-CAL-04.{L}', judge(ok), f'CH{ch} scope PDUTy, median and spread of {N}',
             f'{D_NOM} ±{D_TOL}, spread <= {D_TOL}', fmt(d))
        top, base = m['VTOP'], m['VBASe']
        amp = None if top is None or base is None else top['median'] - base['median']
        spread = None if amp is None else max(top['max'] - top['min'], base['max'] - base['min'])
        ok = amp is not None and abs(amp - V_HIGH) <= AMP_TOL * V_HIGH and spread <= LEVEL_SPREAD
        line(f'L0-CAL-05.{L}', judge(ok), f'CH{ch} amplitude VTOP - VBASe (×10 probe setting)',
             f'{V_HIGH} V ±{AMP_TOL:.0%}, level spread <= {LEVEL_SPREAD * 1e3:.0f} mV',
             'invalid' if amp is None else f'{amp:.3f} V (VTOP {top["median"]:.3f}, VBASe {base["median"]:.3f}, '
                                           f'spread {spread * 1e3:.0f} mV)')
        c['amplitude_V'] = amp
        if ch == 1:
            amp1 = amp
        c['gain_vs_ch1'] = None if amp is None or not amp1 else amp / amp1
        line(f'L0-CAL-06.{L}', 'INFO', f'CH{ch} gain against CH1 (recorded for the campaign)', 'recorded',
             'invalid' if c['gain_vs_ch1'] is None else f'{c["gain_vs_ch1"]:.4f}')
    for i, s in enumerate(skews):
        ch, L = s['channel'], letters[i + 1]
        line(f'L0-CAL-07.{L}', 'INFO', f'Skew CH1 → CH{ch}, rising (50 % crossings from the waveforms, 5 single '
             f'captures, 20 ns/div)', 'recorded; 1 GSa/s, crossings interpolated', fmt(s['rising'], 'ns', 1e9, 3))
        line(f'L0-CAL-08.{L}', 'INFO', f'Skew CH1 → CH{ch}, falling (same method)', 'recorded',
             fmt(s['falling'], 'ns', 1e9, 3))

    cal = {'channels': {f'ch{c["channel"]}': {'amplitude_V': c['amplitude_V'], 'gain_vs_ch1': c['gain_vs_ch1'],
                                              'compensation_pct': c['compensation_pct']} for c in chans},
           'skew_s': {f'ch1_ch{s["channel"]}': {'rising': s['rising'] and s['rising'][0],
                                                'falling': s['falling'] and s['falling'][0]} for s in skews},
           'scope': idn, 'mux_firmware': MUX_FW}
    with open(os.path.join(args.out, 'calibration.json'), 'w') as fh:
        json.dump(cal, fh, indent=1)
    with open(os.path.join(args.out, 'report.json'), 'w') as fh:
        json.dump({'channels': chans, 'skews': skews}, fh)
    man = {'title': 'L0-CAL · SPOC bench calibration',
           'purpose': 'Checks the SPOC bench before any L0 safety case is judged on it (ADR 0009): each scope channel\'s '
                      'divider is trimmed, the scope\'s own timing and level measurements are steady on a known signal '
                      '(the cable check), and the gain and skew of the four channels are recorded for the campaign.',
           'criteria': ['L0-CAL-'],
           'conditions': [['Signal', 'CUT PA8, 1 kHz, duty 0.5, 0 to 3.3 V (Core-CUT /Cut/PwmOut)'],
                          ['Routing', 'PA8 to one channel at a time for 02-06; CH1 + CHn together (route_pair) for the skew'],
                          ['Injection wiring', 'TP1-TP3 wired (self-injection): SPOC CH1 and CH2 share one node, so routing '
                                               'either shows the signal on both scope channels, and the CH1 -> CH2 skew '
                                               'compares only the two dividers and cables (the mux path is shared). Never '
                                               'route two different CUT outputs to CH1 and CH2 while the wire is in'],
                          ['Scope', 'MHO984, ×10 probe setting (9 MΩ divider into 1 MΩ), DC, 1 V/div, NORMal; '
                                    '200 µs/div for 02-06, 20 ns/div and stopped single captures for 07-08'],
                          ['Skew method', '50 % crossing of each channel\'s own swing, interpolated, from the screen '
                                          'waveforms; the scope\'s RRDelay read invalid and FFDelay a constant -0.2 ns'],
                          ['Compensation', 'level 5-25 µs after each edge against the settled level (Core4 '
                                           'analysis.compensation_pct), from the screen waveform'],
                          ['Cables', '30 cm BNC (1 m cables made the scope miss edges, spoc_bench.md)'],
                          ['Not covered here', 'absolute gain against a reference instrument; skew below the 1 ns '
                                               'sample spacing is interpolated by the scope']],
           'target': target_snapshot(idn), 'report': 'report.json', 'figures': 'spoc_cal'}
    with open(os.path.join(args.out, 'target_manifest.json'), 'w') as fh:
        json.dump(man, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
