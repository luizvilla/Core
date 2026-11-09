#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file analyze.py
## @brief Metrics of the inverter bench-sequence runs (L1-GFL-09…13) from runner.py outputs.
## @par Usage
##   python3 tests/l1/design/analyze.py RUN_DIR [RUN_DIR ...]   # markdown table on stdout
## @par Metrics
##   sequence events and faults (control.csv, 10 kHz); contact instant against the ideal grid's zero
##   crossings; closing current peak and residual current from the stored line current; I_d step
##   on the fundamental of the line current (one-period sliding fit) and on the controller's own
##   I_d; I_q; current THD over the last 5 periods.
"""Reads runs, judges nothing by itself: the limits are the catalogue's (tests/catalogue.json)."""
import csv, json, math, sys
from pathlib import Path
import numpy as np

F = 50.0


def events(rows):
    out = []; prev = None
    for q in rows:
        key = tuple(int(float(q[k])) for k in ('inverter_mode', 'relay_close', 'relay_feedback', 'en_bridge', 'inverter_fault', 'bench_trip'))
        if key != prev: out.append((float(q['time_s']), key)); prev = key
    return out


def fundamental(t, i, t0, t1):
    """@return (time, d-axis amplitude, q-axis amplitude) of a one-period sliding fit against the
    ideal grid angle (grid voltage = Vpk·sin(2πFt): d in phase with the voltage)."""
    T = 1 / F; m = (t >= t0 - T) & (t <= t1); tt, ii = t[m], i[m]
    dt = np.median(np.diff(tt)); n = int(round(T / dt)); out = []
    s, c = np.sin(2 * np.pi * F * tt), np.cos(2 * np.pi * F * tt)
    for k in range(n, len(tt), max(1, n // 20)):
        w = slice(k - n, k)
        out.append((tt[k - 1], 2 * np.mean(ii[w] * s[w]), 2 * np.mean(ii[w] * c[w])))
    return np.array(out)


def thd(t, i, t1, periods=5):
    m = t >= t1 - periods / F; x = i[m] - np.mean(i[m]); n = len(x)
    X = np.abs(np.fft.rfft(x * np.hanning(n))); f = np.fft.rfftfreq(n, np.median(np.diff(t[m])))
    k1 = np.argmin(np.abs(f - F)); fund = X[k1 - 1:k1 + 2].max()
    h = [X[np.argmin(np.abs(f - h * F)) - 1:np.argmin(np.abs(f - h * F)) + 2].max() for h in range(2, 41)]
    return float(np.sqrt(np.sum(np.square(h))) / fund) if fund > 0 else math.nan


def analyze(run):
    run = Path(run); man = json.loads((run / 'manifest.json').read_text()); sc = man['scenario']
    rows = list(csv.DictReader(open(run / 'control.csv'))); w = np.load(run / 'waveforms.npz')
    t = w['time']; iac = w['IP(Grid_current)'] if 'IP(Grid_current)' in w else w['IAC']
    ev = events(rows); res = {'run': run.name, 'events': ev}
    cmds = sc['commands']; t_inject = next(q['time_s'] for q in cmds if q['sync_request'] == 4)
    iref = next(q['inject_current'] for q in cmds if q['sync_request'] == 4)
    faults = [(tt, k) for tt, k in ev if k[0] == 7]
    res['fault'] = faults[0] if faults else None
    cmd_on = [tt for tt, k in ev if k[1] == 1]
    if cmd_on:
        t_contact = cmd_on[0] + man['relay']['close_s']
        ph = (2 * np.pi * F * t_contact) % np.pi  # zero crossings of sin at multiples of π
        res['contact_s'] = t_contact; res['contact_zc_deg'] = math.degrees(min(ph, np.pi - ph)) * (1 if ph < np.pi / 2 else -1)
        m = (t >= t_contact) & (t < t_contact + 0.1); res['closing_peak_A'] = float(np.abs(iac[m]).max()) if m.any() else math.nan
        m = (t >= t_contact + 0.1) & (t < t_inject); res['residual_rms_A'] = float(np.sqrt(np.mean(iac[m] ** 2))) if m.any() else math.nan
    if res['fault'] is None or res['fault'][0] > t_inject:
        end = res['fault'][0] if res['fault'] else t[-1]
        fu = fundamental(t, iac, t_inject - 0.05, end)
        tt, idf, iqf = fu[:, 0], fu[:, 1], fu[:, 2]; a = tt >= t_inject; y = idf[a]; ts = tt[a] - t_inject
        if len(y):
            res['final_id_A'] = float(np.mean(y[ts > ts[-1] - 0.02])); res['final_iq_A'] = float(np.mean(iqf[a][ts > ts[-1] - 0.02]))
            res['overshoot_pct'] = max(0.0, 100 * (y.max() - iref) / iref)
            bad = np.where(np.abs(y - iref) > 0.05 * iref)[0]
            res['settle_5pct_s'] = float(ts[bad[-1]]) if len(bad) and bad[-1] < len(y) - 1 else (0.0 if not len(bad) else math.inf)
            r10 = ts[np.argmax(y >= 0.1 * iref)] if (y >= 0.1 * iref).any() else math.nan
            r90 = ts[np.argmax(y >= 0.9 * iref)] if (y >= 0.9 * iref).any() else math.nan
            res['rise_s'] = float(r90 - r10)
            res['pre_step_id_A'] = float(np.abs(idf[(tt > t_inject - 0.05) & (tt < t_inject)]).max())
            res['pre_step_iq_A'] = float(np.abs(iqf[(tt > t_inject - 0.05) & (tt < t_inject)]).max())
        res['thd'] = thd(t, iac, end)
    return res


def main():
    print('| Run | Fault | Contact from zero crossing (°) | Closing peak (A) | Residual (A RMS) | Before step: \\|I_d\\|, \\|I_q\\| (A) | Rise (ms) | Overshoot (%) | Settle 5 % (ms) | Final I_d (A) | Final I_q (A) | THD (%) |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    f = lambda x, s=1, d=3: '—' if x is None or (isinstance(x, float) and not math.isfinite(x)) else f'{x * s:.{d}g}'
    for run in sys.argv[1:]:
        r = analyze(run)
        fault = f"{r['fault'][0]:.4f} s, code {r['fault'][1][4]}{', bench trip' if r['fault'][1][5] else ''}" if r['fault'] else 'none'
        print(f"| {r['run']} | {fault} | {f(r.get('contact_zc_deg'))} | {f(r.get('closing_peak_A'))} | {f(r.get('residual_rms_A'))} | "
              f"{f(r.get('pre_step_id_A'))}, {f(r.get('pre_step_iq_A'))} | {f(r.get('rise_s'), 1e3)} | {f(r.get('overshoot_pct'))} | "
              f"{f(r.get('settle_5pct_s'), 1e3)} | {f(r.get('final_id_A'))} | {f(r.get('final_iq_A'))} | {f(r.get('thd'), 100)} |")


if __name__ == '__main__':
    main()
