#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file recap_2026-11-13_evening.py
## @brief Experimental report of the 2026-11-13 evening bench session (20:00-23:30): one self-contained
##   HTML page (summary table, uPlot figures, MSO54 screenshots, console excerpts, ST-Link reproduction)
##   to send to someone who was not there.
## @par Status
##   First hand-written instance, kept as the prototype for an "export experimental report" tool
##   (see README.md in this folder). The session narrative, the campaigns and the numbers are hard-coded.
## @par Usage
##   python3 tests/common/experimental_report/recap_2026-11-13_evening.py [output.html]
##   Needs the campaigns under tests/out/2026-11-13_* (local, not in git). Default output:
##   docs/testing/recaps/2026-11-13_evening_recap.html
"""Build the self-contained recap page of the 2026-11-13 evening (20:00-23:30)."""
import base64, csv, html, json, re, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / 'tests/out'
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'docs/testing/recaps/2026-11-13_evening_recap.html'

# uPlot (MIT), vendored, inlined so the page works offline
VENDOR = HERE.parent / 'vendor'
UPLOT_JS = '<script>' + (VENDOR / 'uPlot.iife.min.js').read_text() + '</script>'
UPLOT_CSS = '<style>' + (VENDOR / 'uPlot.min.css').read_text() + '</style>'

FIGS = {}
def fig(fid, title, x_label, series, x, scales=None, kind='line', note=''):
    """series: list of dict(label, y, color, scale='y', dash=False)"""
    def clean(a):
        return [None if (v is None or not np.isfinite(v)) else float(f'{v:.6g}') for v in a]
    FIGS[fid] = dict(title=title, xl=x_label, x=clean(x), kind=kind,
                     s=[dict(l=s['label'], y=clean(s['y']), c=s['color'], sc=s.get('scale', 'y'),
                             d=s.get('dash', False), p=s.get('points', False)) for s in series],
                     scales=scales or {'y': ''})
    return (f'<figure><div class="ftitle">{html.escape(title)}</div>'
            f'<div class="iplot" data-fig="{fid}"></div>'
            f'<figcaption>{note} <span class="hint">Drag to zoom · double-click to reset · hover for values · click a legend entry to hide it</span></figcaption></figure>')

def img(path, caption):
    b = base64.b64encode((OUT / path).read_bytes()).decode()
    return (f'<figure class="shot"><a href="data:image/png;base64,{b}" target="_blank">'
            f'<img loading="lazy" src="data:image/png;base64,{b}" alt="{html.escape(caption)}"></a>'
            f'<figcaption>{caption} <span class="hint">MSO54 screenshot, <code>tests/out/{path}</code></span></figcaption></figure>')

C1, C2, C3, C4, C5 = 'var(--c1)', 'var(--c2)', 'var(--c3)', 'var(--c4)', 'var(--c5)'

def npz(p):
    return np.load(OUT / p / 'waveforms.npz')

def period_mean(t, y, T=20e-6):
    """block mean over one switching period: removes the 50 kHz ripple"""
    n = int(round(T / (t[1] - t[0])))
    m = len(y) // n * n
    return t[:m].reshape(-1, n).mean(1), y[:m].reshape(-1, n).mean(1)

def dump(p):
    rows = list(csv.DictReader(open(OUT / p)))
    g = lambda k: np.array([float(r[k]) for r in rows])
    return {k: g(k) for k in rows[0]}

def harmonics(t, y, f=50.0, kmax=7):
    n = int((t[-1] - t[0]) * f); m = t < t[0] + n / f; tt, yy = t[m], y[m]
    return [float(np.mean(yy))] + [float(abs(2 * np.mean(yy * np.exp(-2j * np.pi * k * f * tt)))) for k in range(1, kmax + 1)]

parts = []
P = parts.append

# ---------------------------------------------------------------- L2-INV-09
dts = [100, 160, 220, 220, 300]
h100 = [0.247, 0.258, 0.236, 0.240, 0.213]
h250 = [0.152, 0.175, 0.187, 0.184, 0.221]
f09a = fig('inv09h', 'L2-INV-09 · line current harmonics against the bridge dead time', 'dead time (ns)',
           [dict(label='100 Hz (A peak)', y=h100, color=C1, points=True),
            dict(label='250 Hz (A peak)', y=h250, color=C2, points=True)], dts, kind='pts',
           note='CH3, valid captures 20:47 and 20:52–20:55 (worklog numbers). The 250 Hz rises with the dead time, the 100 Hz does not.')
S09 = '2026-11-13_175f50e_204629_bench/l2_inv09/captures/'
a = npz(S09 + '204720_dt100'); b = npz(S09 + '205247_dt300')
ta, ya = period_mean(a['CH3_t'], a['CH3_v']); tb, yb = period_mean(b['CH3_t'], b['CH3_v'])
f09b = fig('inv09w', 'L2-INV-09 · line current (CH3), averaged over each 20 µs switching period', 'time (ms)',
           [dict(label='dt 100 ns (A)', y=ya, color=C1), dict(label='dt 300 ns (A)', y=np.interp(ta, tb, yb), color=C2)],
           ta * 1e3, note='Connected, Kp 3.5 V/A, Ti 20 ms, measured feedforward (ff 0), filter off.')

# ---------------------------------------------------------------- steady dump 211235
d = dump('2026-11-13_24e7c97_205616_bench/firmware_scope/211235_firmware_scope.csv')
ts = np.arange(len(d['VAC_V'])) / 2000 * 1e3
vbr = (2 * d['bridge_duty'] - 1) * d['VDCBus_V']
fdump = fig('dump211235', 'Firmware scope dump 21:12:35 (2 kHz, connected, filter off): node VAC, bridge command, IAC', 'time (ms)',
            [dict(label='VAC (V)', y=d['VAC_V'], color=C1),
             dict(label='bridge (2d−1)·Vbus (V)', y=vbr, color=C2, dash=True),
             dict(label='IAC (A)', y=d['IAC_A'], color=C3, scale='A')], ts, scales={'y': 'V', 'A': 'A'},
            note='VAC and the bridge command lie on top of each other: at 100 Hz the bridge copies VAC with gain 0.965 at +0.3°. The loop VAC → feedforward → bridge → VAC sustains the 100 Hz and the DC.')
# zoom on the difference
fdiff = fig('dump211235d', 'Same dump · VAC minus bridge command (what drives the current through the inverter impedance)', 'time (ms)',
            [dict(label='VAC − bridge (V)', y=d['VAC_V'] - vbr, color=C4), dict(label='IAC (A)', y=d['IAC_A'], color=C3, scale='A')],
            ts, scales={'y': 'V', 'A': 'A'})

# ---------------------------------------------------------------- L2-INV-11
S11 = '2026-11-13_cb36de1_212700_bench/l2_inv11/captures/'
off = npz(S11 + '212827_ffn0'); on = npz(S11 + '212908_ffn1')
to, yo = period_mean(off['CH3_t'], off['CH3_v']); tn, yn = period_mean(on['CH3_t'], on['CH3_v'])
f11w = fig('inv11w', 'L2-INV-11 · line current (CH3) filter off / on, averaged over each switching period', 'time (ms)',
           [dict(label='filter off, ffn 0 (A)', y=yo, color=C1), dict(label='filter on, ffn 1 (A)', y=np.interp(to, tn, yn), color=C2)],
           to * 1e3, note='Captures 21:28:27 (off) and 21:29:08 (on), 7 s after G.')
ho = harmonics(off['CH3_t'], off['CH3_v']); hn = harmonics(on['CH3_t'], on['CH3_v'])
lab = ['DC*', '50', '100', '150', '200', '250', '300', '350']
f11h = fig('inv11h', 'L2-INV-11 · line current spectrum (A peak; DC = scope mean, includes the ≈0.03 A probe offset)', 'harmonic (Hz)',
           [dict(label='filter off', y=ho, color=C1), dict(label='filter on', y=hn, color=C2)],
           list(range(8)), kind='bars', note='x positions: ' + ', '.join(f'{i} = {l}' for i, l in enumerate(lab)) + ' Hz. 100 Hz 0.253 → 0.012 A (21×); odd harmonics unchanged, as designed.')
don = dump('2026-11-13_cb36de1_212700_bench/firmware_scope/213024_firmware_scope.csv')
doff = dump('2026-11-13_cb36de1_212700_bench/firmware_scope/213041_firmware_scope.csv')
def fs_of(p):
    m = re.search(r'(\d+)\s*Hz', (OUT / p).read_text()); return float(m.group(1)) if m else 2000.0
fsn = 2000.0
tdn = np.arange(len(don['VAC_V'])) / fsn * 1e3
f11d = fig('inv11d', 'L2-INV-11 · firmware dumps: VAC − bridge command, filter on (21:30:24) and off (21:30:41)', 'time (ms)',
           [dict(label='filter off: VAC − bridge (V)', y=doff['VAC_V'] - (2 * doff['bridge_duty'] - 1) * doff['VDCBus_V'], color=C1),
            dict(label='filter on: VAC − bridge (V)', y=don['VAC_V'] - (2 * don['bridge_duty'] - 1) * don['VDCBus_V'], color=C2)], tdn,
           note='The filter-on dump was taken 0.7 s after the contact (end of the closing transient). Time axis assumes 2 kHz.')

# ---------------------------------------------------------------- L2-INV-12 console vdc
def console(p):
    rows = []
    for line in open(OUT / p):
        m = re.match(r'(\S+) ([<>]) (.*)', line.rstrip('\n'))
        if not m: continue
        rows.append((m.group(1), m.group(2), m.group(3)))
    return rows
from datetime import datetime
def tel(rows, t0):
    out = []
    for ts, d_, txt in rows:
        if d_ != '<' or not txt.startswith('st '): continue
        tok = txt.split(); kv = {tok[i]: tok[i + 1] for i in range(0, len(tok) - 1, 2)}
        out.append(((datetime.fromisoformat(ts) - t0).total_seconds(), kv))
    return out
r12 = console('2026-11-13_e5884e2_215115_bench/console.txt')
t0 = datetime.fromisoformat('2026-11-13T21:52:00')
T12 = [r for r in tel(r12, t0) if 0 <= r[0] <= 110]
x12 = [a for a, _ in T12]
f12 = fig('inv12', 'L2-INV-12 · telemetry: DC-loop correction vdc and line current IAC (seconds after 21:52:00)', 's',
          [dict(label='vdc, DC loop output (V)', y=[float(k.get('vdc', 'nan')) for _, k in T12], color=C1),
           dict(label='IAC (A)', y=[float(k.get('IAC', 'nan')) for _, k in T12], color=C3, scale='A'),
           dict(label='relay (rly 11 = 1)', y=[1.0 if k.get('rly') == '11' else 0.0 for _, k in T12], color=C5, scale='A', dash=True)],
          x12, scales={'y': 'V', 'A': 'A'},
          note='First connection (21:52:10, B off), then B pressed at 21:52:50 and connected again at 21:53:10: vdc settles at −5.2 to −5.4 V.')

# ---------------------------------------------------------------- L2-INV-13 ripple and replay
c13 = npz('2026-11-13_c862597_222057_bench/l2_inv13/captures/222304_adt0.00')
t, y = c13['CH3_t'], c13['CH3_v']
m = (t >= 0.0101) & (t < 0.0101 + 100e-6)
f13w = fig('inv13w', 'L2-INV-13 · line current at full rate over 100 µs (five switching periods)', 'time (µs)',
           [dict(label='CH3 (A)', y=y[m][::4], color=C1)], (t[m][::4] - t[m][0]) * 1e6,
           note='344 mA at 50 kHz: one ADC sample per period at a fixed instant reads part of this ripple as DC.')
fsw = 49998.8; T = 1 / fsw; N = int((t[-1] - t[0]) / T) - 2
true = np.mean(y[(t >= t[0]) & (t < t[0] + N * T)])
ph = np.linspace(0, 1, 201)[:-1]
def err(p):
    return np.interp(t[0] + (np.arange(N) + p) * T, t, y).mean() - true
E1 = np.array([err(p) for p in ph])
E2 = np.array([(err(p) + err((p + .5) % 1)) / 2 for p in ph])
E4 = np.array([np.mean([err((p + q) % 1) for q in (0, .25, .5, .75)]) for p in ph])
f13r = fig('inv13r', 'L2-INV-13 · DC error of the IAC average against the sampling phase (replay of the CH3 record)', 'sampling phase in the switching period',
           [dict(label='1 sample / period (mA)', y=E1 * 1e3, color=C1), dict(label='2 samples, T/2 apart (mA)', y=E2 * 1e3, color=C2),
            dict(label='4 samples, T/4 apart (mA)', y=E4 * 1e3, color=C3)], ph,
           note=f'Recomputed here: one sample {E1.min()*1e3:.0f} to +{E1.max()*1e3:.0f} mA, two samples {E2.min()*1e3:.0f} to +{E2.max()*1e3:.0f} mA. This chose ADR 0008 (two samples, EdgeTrigger_Both).')

# ---------------------------------------------------------------- incident console
rI = console('2026-11-13_019d33c_223503_bench/console.txt')
t0I = datetime.fromisoformat('2026-11-13T22:36:00')
TI = [r for r in tel(rI, t0I) if -5 <= r[0] <= 35]
xI = [a for a, _ in TI]
fI = fig('incident', 'Incident session · telemetry before the freeze (seconds after 22:36:00)', 's',
         [dict(label='IAC (A)', y=[float(k['IAC']) for _, k in TI], color=C3, scale='A'),
          dict(label='iacm, 0.5 s mean (A)', y=[float(k['iacm']) for _, k in TI], color=C2, scale='A'),
          dict(label='Vbus − 400 (V)', y=[float(k['Vbus']) - 400 for _, k in TI], color=C1),
          dict(label='relay (rly 11 = 1)', y=[1.0 if k['rly'] == '11' else 0.0 for _, k in TI], color=C5, scale='A', dash=True)],
         xI, scales={'y': 'V', 'A': 'A'},
         note='Normal operation for 28 s connected (IAC within ±0.4 A, Vbus 400–402 V, flt 0, load 35–36 µs, iacm 7–8 mA). The last line is at 32.73 s; <code>r</code> was sent at 32.45 s.')
key_lines = [l for l in open(OUT / '2026-11-13_019d33c_223503_bench/console.txt') if ' > ' in l or 'scope' in l or 'begin record' in l]
last_lines = open(OUT / '2026-11-13_019d33c_223503_bench/console.txt').read().splitlines()[-9:]
tim_last = open(OUT / '2026-11-13_cd277eb_225411_Go83KY/l0_tim03/console.txt').read().splitlines()[-4:]
tim_log = open(OUT / '2026-11-13_cd277eb_225411_Go83KY/l0_timing.log').read().splitlines()[-5:]

# ---------------------------------------------------------------- screenshots
SH = {
 '09a': img(S09 + '204720_dt100/screen.png', 'L2-INV-09, dead time 100 ns (20:47:20). This screenshot came from a later sweep, after the relay opened (the capture defect fixed in 24e7c97); the waveform data plotted above is valid.'),
 '09b': img(S09 + '205247_dt300/screen.png', 'L2-INV-09, dead time 300 ns, connected (20:52:47).'),
 '11a': img(S11 + '212827_ffn0/screen.png', 'L2-INV-11, feedforward filter off (21:28:27).'),
 '11b': img(S11 + '212908_ffn1/screen.png', 'L2-INV-11, feedforward filter on (21:29:08).'),
 '12a': img('2026-11-13_e5884e2_215115_bench/l2_inv12/captures/215236_dcl0/screen.png', 'L2-INV-12, DC loop off (21:52:36).'),
 '12b': img('2026-11-13_e5884e2_215115_bench/l2_inv12/captures/215328_dcl1/screen.png', 'L2-INV-12, DC loop on (21:53:28).'),
 '13a': img('2026-11-13_c862597_222057_bench/l2_inv13/captures/222304_adt0.00/screen.png', 'L2-INV-13 first run, one IAC sample per period at the valley (22:23:04).'),
}

esc = html.escape
TABLE = [
 ('20:00–20:15', 'L2-INV-08 · gain keys (+ − &gt; &lt;), bumpless PI gain change; D-GFL-10 averaged gain sweep',
  '<code>3b097e5</code>, <code>4a4ad6c</code>', 'Raising Kp (3.5 → 7 V/A) reduces the connected residual current',
  'No effect: the steady part of Id, Iq ≈ 0.04 A with both gain sets, fluctuation 0.33 A RMS. Line current: 0.26 A at <b>100 Hz</b>, 0.013 A at 50 Hz, 0.08–0.13 A at 150/250/350 Hz, 0.32 A at 50 kHz. The 100 Hz is absent with the relay open.', 'rejected'),
 ('20:44–20:55', 'L2-INV-09 · bridge dead time from keys w / s (100, 160, 220, 300 ns)', '<code>175f50e</code>, fix <code>24e7c97</code>',
  'Dead time is the source of the 100 Hz', 'The 250 Hz rises +45 % with the dead time, the 100 Hz stays 0.21–0.26 A. <b>Dead time is not the source.</b> Side defect: the MSO54 capture read channels from different sweeps; fixed (hold the acquisition during a capture).', 'rejected'),
 ('21:00–21:14', 'Firmware scope dumps while connected (10 kHz, then 2 kHz)', 'fix <code>336963a</code>',
  'The 100 Hz comes from the control path, not the hardware', 'The bridge command copies the node VAC at 100 Hz with gain 0.965 at +0.3°: the measured feedforward closes a loop VAC → bridge → VAC with gain ≈ 1, so any small 100 Hz / DC asymmetry is sustained (amplified ≈ 9×). The measured DC bus in the duty is not the source (0.2 V of 9.2 V). Side fix: scope re-arm lost the pre-trigger samples.', 'confirmed'),
 ('21:25–21:32', 'L2-INV-11 · feedforward DC block + 100 Hz notch (key D), library <code>0cce694</code>', '<code>cb36de1</code>',
  'Filtering DC and 2ω out of the feedforward removes them (predicted ≈ 10×)', '<b>100 Hz 0.253 → 0.012 A (21×)</b>, DC 126 → 10 mA on the DC ammeter (12.6×), fundamental untouched (bridge/VAC at 50 Hz 1.015, +1.8° both). Odd harmonics (≈ 0.15 A at 250/350 Hz) remain.', 'confirmed'),
 ('21:49–21:57', 'L2-INV-12 · DC current loop (key B) with sensor auto-zero, library <code>67097f2</code>', '<code>e5884e2</code>',
  'An integral loop on the IAC mean drives the line DC to zero', 'Loop stable and correctly signed (vdc settles at −5.3 V, τ ≈ 1 s) but the true DC goes to <b>−140 mA</b> (ammeter): the firmware IAC reads <b>+145 mA too high while connected</b> (+6 mA open).', 'new defect'),
 ('22:15–22:33', 'L2-INV-13 · IAC sampling instant (keys [ / ], iacm); then ADR 0008: two IAC samples per period (EdgeTrigger_Both, CMP3 = 0.5)', '<code>c862597</code>, <code>019d33c</code>',
  'The IAC bias is the 50 kHz ripple sampled at a fixed phase', 'First run (one sample at the valley): iacm +137 mA vs true −11 mA. Replay: one sample per period errs −544…+534 mA depending on phase, two samples T/2 apart −8…+11 mA. Two samples implemented; host L0 166 pass, 0 new fail.', 'confirmed'),
 ('22:35–22:39', '<b>Incident</b> · L2-INV-13 with firmware <code>019d33c</code>, 400 V bus, mains through a 100 W bulb', '<code>019d33c</code>',
  '—', '28 s connected, normal; iacm 7–8 mA (bias gone). <code>q</code>, then <code>r</code> (scope dump) at 22:36:32.45: console prints "begin record" and <b>nothing after</b>. Loud explosion at about that moment; bulb lit, bottom-left bridge transistor glowing red, bus collapsed fast after the source was switched off. Both transistors of one leg blown open.', 'board lost'),
 ('22:54–23:05', 'L0-TIM-03 on the bare Spin over ST-Link (current code, then the <code>c862597</code> firmware files)', '<code>cd277eb</code>',
  'Is the console freeze at <code>r</code> reproducible without the power stage?', '<b>Yes, twice.</b> Tick counter frozen at 807 122 at the first <code>r</code> during TIM-03. Not ADR 0008 (pre-change firmware freezes too); at <code>ef4be85</code> (17:13) every dump completed.', 'reproduced'),
 ('23:05–23:21', 'SWD post-mortem of the frozen core (OpenOCD + GDB)', '<code>f7cf4316</code>, <code>5b54bbe</code>',
  'What stops the core?', '<b>K_ERR_STACK_CHK_FAIL</b>: the background (asynchronous) task overflows its 1 KB stack in newlib <code>sprintf</code> inside <code>ScopeMimicry::dump_datas()</code> when the 20 kHz interrupt stacks its FPU frame on top. Zephyr halts with interrupts locked: control, 3 A trip, console stop; the HRTIM keeps switching at its last duty with the relay closed.', 'root cause (firmware)'),
]
STATUS = {'rejected': 'bad', 'confirmed': 'good', 'new defect': 'warn', 'board lost': 'bad', 'reproduced': 'warn', 'root cause (firmware)': 'good'}
rows = ''.join(f'<tr><td class="nw">{a}</td><td>{b}</td><td class="nw">{c}</td><td>{d}</td><td>{e}</td><td><span class="badge {STATUS[f]}">{f}</span></td></tr>'
               for a, b, c, d, e, f in TABLE)

def pre(lines): return '<pre>' + esc('\n'.join(l.rstrip('\n') for l in lines)) + '</pre>'

BODY = f'''
<header>
<h1>Inverter bench, Friday 13 Nov 2026, 20:00–23:30</h1>
<p class="sub">Grid-following inverter on the OwnTech Spin (uSolarVerter shield): from a 100 Hz line current to a blown bridge leg and its firmware root cause.
Branch <code>apf_quadrature</code> of <code>luizvilla/Core</code>, commits <code>3b097e5</code> … <code>5b54bbe</code>. Source of everything below: <code>docs/testing/worklog.md</code> (entries of 2026-11-13) and the campaigns under <code>tests/out/2026-11-13_*</code>.</p>
</header>

<section class="tldr">
<h2>In three lines</h2>
<ol>
<li>The 100 Hz (and DC) line current while connected came from the <b>measured feedforward copying the inverter's own node voltage</b> (loop gain ≈ 1). A DC block + 100 Hz notch on the feedforward removed it (21×). Neither PI gains nor dead time were the cause.</li>
<li>The firmware IAC read +145 mA too high because <b>one ADC sample per period aliases the 50 kHz ripple into DC</b>; two samples half a period apart fix it (ADR 0008, confirmed on the bench: iacm 7–8 mA).</li>
<li>The board blew a bridge leg at the moment a <b>scope dump</b> was requested. On the bare Spin the same dump makes the background task <b>overflow its 1 KB stack</b>; Zephyr halts the core with every software protection off while the PWM keeps running. How a halted controller became a shoot-through in a fraction of a second is <b>still open</b>: that is the test to run (section 4).</li>
</ol>
</section>

<h2>1 · What was tried, in order</h2>
<div class="wrap"><table class="main">
<thead><tr><th>Time</th><th>Step</th><th>Commit</th><th>Hypothesis</th><th>Finding</th><th>Outcome</th></tr></thead>
<tbody>{rows}</tbody></table></div>

<h2>2 · Evidence per step</h2>

<h3>2.1 Dead time is not the source of the 100 Hz (L2-INV-09)</h3>
<p>Bench: 400 V bus, mains through a 100 W bulb, Kp 3.5 V/A, Ti 20 ms, measured feedforward. One connected capture per dead time. MSO54 channels: CH1 inverter side of the relay, CH2 grid side, CH3 line current, CH4 relay command.</p>
{f09a}
{f09b}
<div class="pair">{SH['09a']}{SH['09b']}</div>

<h3>2.2 The bridge copies the node's 100 Hz (firmware scope dump)</h3>
{fdump}
{fdiff}
<table class="kv"><tr><td>Six periods, 100 Hz</td><td>VAC 9.52 V (+53°), DC −3.87 V · bridge 9.18 V (+53°), DC −4.00 V · IAC 0.144 A (+29°), DC +0.038 A · DC bus 0.50 V</td></tr>
<tr><td>Ratios at 100 Hz</td><td>bridge / VAC = 0.965 at +0.3° · VAC / IAC = 66 Ω at +24°</td></tr>
<tr><td>Reading</td><td>Behind ≈ 5 Ω of inverter impedance and 66 Ω of bulb path, an internal 100 Hz source is amplified ≈ 66 / (66·0.035 + 5) ≈ 9×: about 1 V inside the inverter gives the 9.5 V and 0.14 A observed.</td></tr></table>

<h3>2.3 Feedforward filter removes the 100 Hz and the DC (L2-INV-11)</h3>
<p>Library <code>luizvilla/single_phase_inverter@0cce694</code>: on the measured feedforward, first-order DC block (τ 0.2 s) and a notch at 2ω (input minus a SOGI at 2ω), then a complex gain that restores the fundamental exactly. Key <code>D</code>.</p>
{f11w}
{f11h}
{f11d}
<div class="pair">{SH['11a']}{SH['11b']}</div>

<h3>2.4 DC loop works, IAC is biased (L2-INV-12)</h3>
<table class="kv"><thead><tr><th>DC per source</th><th>loop off</th><th>loop on</th></tr></thead>
<tr><td>DC ammeter in series</td><td>−10 mA</td><td>−140 mA</td></tr>
<tr><td>Scope CH3 (firmware sign, minus 0.03 A probe offset)</td><td>−12 mA</td><td>−143 mA</td></tr>
<tr><td>Firmware IAC mean</td><td>+134 mA</td><td>+5 mA</td></tr>
<tr><td>Bridge DC</td><td>−0.56 V</td><td>−5.31 V</td></tr></table>
{f12}
<div class="pair">{SH['12a']}{SH['12b']}</div>

<h3>2.5 The IAC bias is ripple aliasing (L2-INV-13, ADR 0008)</h3>
{f13w}
{f13r}
<div class="pair">{SH['13a']}<div></div></div>
<p>Change in <code>019d33c</code>: overlay LEG2_HIGH <code>default-edge-trigger = "EdgeTrigger_Both"</code> (CMP3 triggers ADC_2 counting up and down, RM0440 §27.5), CMP3 = 0.5 at start-up, so IAC is sampled at T/4 and 3T/4. Gate outputs are set/reset by CMP1 only in up-down mode; CMP3/ADROM only trigger the ADC.</p>

<h2>3 · The incident and its firmware root cause</h2>
<h3>3.1 Bench console around the explosion (<code>tests/out/2026-11-13_019d33c_223503_bench/console.txt</code>)</h3>
{fI}
<p>Keys sent and scope lines:</p>
{pre(key_lines)}
<p>Last lines of the console:</p>
{pre(last_lines)}
<table class="kv">
<tr><td>Settings</td><td>Firmware <code>019d33c</code>, Kp 3.5 V/A, Ti 20 ms, dead time 100 ns, measured feedforward (ff 0), filter on (ffn 1), DC loop off (dcl 0), adt 0.50. 400 V / 0.5 A DC source (not in current limit), 100 W bulb in series with the mains.</td></tr>
<tr><td>Operators saw</td><td>A sudden, very loud explosion very near the moment <code>r</code> was pressed. The bulb lit. After disconnecting the grid, smoke kept coming from the bridge and the bottom-left transistor glowed red. After switching off the 400 V source the bus voltage fell very fast. 16 V supply not shorted. Both transistors of one bridge leg blown open.</td></tr>
<tr><td>Firmware check</td><td>Only changes since the 22:20 session: ADC_2 trigger on both counting phases and CMP3 = 0.5. Dead time (DTxR) written only at start-up and by keys w/s (not pressed; telemetry dt 100 throughout). No code path found that changes the dead time or gate pattern.</td></tr>
</table>

<h3>3.2 Reproduced on the bare Spin: L0-TIM-03 freezes at the first scope dump</h3>
<table class="kv"><thead><tr><th>Campaign</th><th>Firmware</th><th>Result</th></tr></thead>
<tr><td><code>2026-11-13_cd277eb_225411_Go83KY</code></td><td>current code (two-sample IAC)</td><td>froze at the first <code>r</code> during TIM-03 (115.766 s), ticks stuck at 807 122</td></tr>
<tr><td><code>2026-11-13_cd277eb-dirty_230231_ZpGJDI</code></td><td><code>c862597</code> firmware files (before ADR 0008)</td><td>froze the same way (110.455 s)</td></tr>
<tr><td><code>2026-11-13_ef4be85-dirty_171310_z8S8pr</code> (reference)</td><td><code>ef4be85</code></td><td>every dump during TIM-03 completed</td></tr></table>
<p>Harness log tail, and the last console lines (the dump's channel-name header is the last output):</p>
{pre(tim_log)}
{pre(tim_last)}

<h3>3.3 SWD post-mortem</h3>
<pre>[stm32g4x.cpu] halted due to debug-request, current mode: Handler MemManage
pc 0x0802d15a  lr 0x0802d175  msp 0x20012748
ICSR 0xe000ed04 = 0x0440f804   (VECTACTIVE 4: MemManage)
#0 arch_system_halt (reason=2)              kernel/fatal.c:30
#1 k_sys_fatal_error_handler (reason=2)     kernel/fatal.c:44
#2 z_fatal_error (reason=2)                 kernel/fatal.c:119
#3 z_arm_fault                              arch/arm/core/cortex_m/fault.c:1107
_kernel.cpus[0].current = tasks_information+24, stack_info {{start 0x20011280, size 0x400}}
esf.pc 0x08032c60 = __sflush_r (newlib), esf.r1 in encode_float (cbprintf), r2 in _vfiprintf_r</pre>
<table class="kv">
<tr><td>Reason 2</td><td><code>K_ERR_STACK_CHK_FAIL</code>: the OwnTech asynchronous (background) task, 1 KB stack (<code>CONFIG_OWNTECH_TASK_ASYNCHRONOUS_TASKS_STACK_SIZE=1024</code>, MPU stack guard on).</td></tr>
<tr><td>Mechanism</td><td><code>loop_application_task()</code> (frame 268 B + saved registers) → <code>dump_scope_datas()</code> → <code>printk</code> + newlib <code>sprintf</code>. When the 20 kHz control interrupt lands at the deepest point, the core stacks its exception frame with the FPU context (104 B, <code>CONFIG_FPU_SHARING</code>) on the thread stack and the guard trips.</td></tr>
<tr><td>Why today</td><td>Inference: the telemetry <code>printk</code> grew from 14 arguments (6 doubles, <code>ef4be85</code>) to 21 (<code>e5884e2</code>, dumps still passed at 21:52) and 23 (11 doubles, <code>c862597</code>), enlarging the frame the dump runs on top of. Whether it overflows depends on where the interrupt lands, so a pass proves no margin.</td></tr>
<tr><td>After the fault</td><td>Default Zephyr path <code>k_sys_fatal_error_handler → arch_system_halt</code>, no reset, interrupts locked. Control task, 3 A trip, every software protection and the console stop; the HRTIM keeps switching at its last compare values, with its dead time; the relay stays closed.</td></tr>
<tr><td>Open</td><td>The electrical path from a halted controller to a leg failure within a fraction of a second. Candidates: what the gate signals actually do at the fault instant; a large DC voltage at the frozen duty driving current through the filter and the bulb with no trip; a device failing under that stress and the leg then shooting through.</td></tr>
</table>

<h2>4 · Reproduce it with an ST-Link (no power stage needed)</h2>
<p><b>Only on a bare Spin</b> (no shield, or an unpowered one): the TIMING firmware switches the PWM outputs.</p>
<h3>4.1 Hardware and tools</h3>
<table class="kv">
<tr><td>Board</td><td>Spin 1.2.0, powered from its USB-C (the USB-C is also the console)</td></tr>
<tr><td>Probe</td><td>STLINK-V3 (V2 / V2-1 also work), SWD cable to the Spin's SWD header (SWDIO, SWCLK, GND, VTref)</td></tr>
<tr><td>Software</td><td>PlatformIO Core 6.x in <code>~/.platformio/penv</code>; its <code>tool-openocd</code> (xPack 0.12.0) and <code>toolchain-gccarmnoneeabi</code> 12.3.1; Python ≥ 3.8; MCUboot v1.1.0 on the Spin (preflight installs it). Linux: PlatformIO udev rules (sudo).</td></tr>
<tr><td>Libraries</td><td>Pinned in <code>src/app.ini</code>, cloned by the first build: <code>luizvilla/single_phase_inverter@67097f2</code>, <code>luizvilla/control_library@d1449a4</code>, owntech scopemimicry and examples</td></tr></table>

<h3>4.2 Freeze the core (L0-TIM-03)</h3>
<pre>git clone git@github.com:luizvilla/Core.git &amp;&amp; cd Core
git checkout apf_quadrature            # freezing code: 5b54bbe (firmware identical to cd277eb)
bash tests/l0/timing/preflight.sh      # act on each MISSING line; --install fixes the no-sudo ones
bash tests/l0/timing/run.sh --run      # builds env TIMING, flashes over the ST-Link, runs TIM-01/02/03</pre>
<p>Expected: during TIM-03 the host sends <code>h</code> every 5 s and <code>r</code> every 10 s; at the first <code>r</code> the console stops after the CSV header line, then <code>read_report: RUNNING_TIM03, 807122 ticks</code> repeats until the timeout and the verdict is <code>L0-TIM-01.a FAIL … state RUNNING_TIM03</code>. The exact tick count may differ. <b>Do not power-cycle the board</b> before 4.3.</p>
<p>Contrast: <code>git checkout ef4be85</code> and rerun: every dump completes (one passing run; since the overflow depends on interrupt timing, repeat to be sure).</p>

<h3>4.3 Read the fault over SWD (board still frozen, same <code>firmware.elf</code>)</h3>
<pre>O=~/.platformio/packages/tool-openocd
# 1. Mode and active exception (ICSR VECTACTIVE: 3 HardFault, 4 MemManage, 5 BusFault, 6 UsageFault)
$O/bin/openocd -s $O/openocd/scripts -f interface/stlink.cfg -f target/stm32g4x.cfg \\
  -c init -c halt -c "reg pc" -c "mdw 0xE000ED04 1" -c resume -c shutdown
# 2. Fatal reason, faulting thread and context (OpenOCD as GDB server, then GDB)
$O/bin/openocd -s $O/openocd/scripts -f interface/stlink.cfg -f target/stm32g4x.cfg -c init -c halt &amp;
~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb -batch -nx .pio/build/TIMING/firmware.elf \\
  -ex "target extended-remote :3333" -ex bt -ex "frame 1" -ex "p *esf" \\
  -ex "p _kernel.cpus[0].current" -ex "p/x _kernel.cpus[0].current-&gt;stack_info" -ex "monitor shutdown"
# 3. Symbolize the faulting PC
~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-addr2line -f -e .pio/build/TIMING/firmware.elf &lt;esf.pc&gt;</pre>
<p><code>k_sys_fatal_error_handler(reason)</code>: 0 CPU exception, 2 <code>K_ERR_STACK_CHK_FAIL</code>, 3 kernel oops, 4 kernel panic. <code>stack_info.size = 0x400</code> identifies an OwnTech asynchronous task (the background task).</p>

<h3>4.4 Same with the production (USB) firmware, by hand</h3>
<pre>pio run -e USB -t upload               # the image that ran on the bench (019d33c logic)
# USB console: press  q  then  r  repeatedly; a freeze shows as the console stopping after "begin record"</pre>
<p><b>Not tried yet.</b> On the bench the freeze came at the first <code>r</code> after a <code>q</code> while connected, where the control interrupt does the most work; on a bare board in OFF it may need many tries or not happen. The TIMING run (4.2) is the reproduction that has worked twice. After any TIMING run, flash env USB again before a bench session.</p>

<h3>4.5 The decisive test (not yet done)</h3>
<p>With the TIMING firmware freezing the core with the PWM running (TIM-03, first dump): put the MSO54 on both gate signals of each bridge leg — LEG2_HIGH = PWME, Spin pins 10/11; LEG1_HIGH = PWMD, pins 5/6 — trigger on the dump (console <code>r</code>), and check the dead-time gap and the output states before, at and after the fault. Then the same with the USB firmware and a forced fault.</p>

<h3>4.6 Planned fixes (before any power)</h3>
<ul>
<li>Larger background-task stack, and measure the margin (<code>CONFIG_INIT_STACKS</code>, <code>k_thread_stack_space_get</code>; e.g. ≥ 256 B free after dumps during the critical task).</li>
<li>A fatal-error handler that stops every HRTIM output and opens the relay before halting.</li>
<li>A protection that does not depend on the processor: HRTIM fault input from an overcurrent comparator and/or a watchdog that disables the bridge outputs.</li>
<li>L0-TIM-03 additions: stack-margin criterion, decode the fatal reason over SWD automatically when the ticks stop, back-to-back dumps with full telemetry.</li>
</ul>

<h2>5 · Where the files are</h2>
<table class="kv">
<tr><td>Narrative</td><td><code>docs/testing/worklog.md</code>, entries "2026-11-13 — …" from "Bridge dead time from the bench keys" to "Incident addendum"</td></tr>
<tr><td>Freeze diagnosis how-to</td><td><code>tests/l0/timing/README.md</code>, section "When the harness does not finish (frozen target)"</td></tr>
<tr><td>Bench sessions</td><td><code>tests/out/2026-11-13_175f50e_204629_bench</code> (INV-09), <code>…_24e7c97_205616_bench</code> (dumps), <code>…_cb36de1_212700_bench</code> (INV-11), <code>…_e5884e2_215115_bench</code> (INV-12), <code>…_c862597_222057_bench</code> (INV-13 first run), <code>…_019d33c_223503_bench</code> (incident)</td></tr>
<tr><td>L0-TIM freezes</td><td><code>tests/out/2026-11-13_cd277eb_225411_Go83KY</code>, <code>…_cd277eb-dirty_230231_ZpGJDI</code>, each with <code>l0_tim03/fatal_diagnosis.md</code> and <code>l0_tim03/console.txt</code></td></tr>
<tr><td>Decisions</td><td>ADR 0008 (two IAC samples per period); ADR 0005 (on-target timing tests)</td></tr>
<tr><td>Note</td><td><code>tests/out/</code> is not committed: the raw captures above are on Luiz's machine; the plots and screenshots in this page are the copies that travel.</td></tr>
</table>
<p class="sub">Generated 2026-11-15 from the repository data. Plots: uPlot, data embedded.</p>
'''

PAGE = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Friday Bench Recap</title>
<style>
:root{{--bg:#fcfcfb;--ink:#141413;--ink2:#5c5b57;--line:#e4e2dc;--card:#fff;--code:#f2f1ed;
--c1:#2a6fdb;--c2:#d9480f;--c3:#2b8a3e;--c4:#7048e8;--c5:#868e96;--good:#2b8a3e;--bad:#c92a2a;--warn:#b35c00}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#181817;--ink:#ecebe7;--ink2:#a7a59e;--line:#34332f;--card:#201f1d;--code:#2a2926;
--c1:#6ea8fe;--c2:#ff8c42;--c3:#5fd08a;--c4:#b197fc;--c5:#adb5bd;--good:#5fd08a;--bad:#ff6b6b;--warn:#ffb347}}}}
:root[data-theme="dark"]{{--bg:#181817;--ink:#ecebe7;--ink2:#a7a59e;--line:#34332f;--card:#201f1d;--code:#2a2926;
--c1:#6ea8fe;--c2:#ff8c42;--c3:#5fd08a;--c4:#b197fc;--c5:#adb5bd;--good:#5fd08a;--bad:#ff6b6b;--warn:#ffb347}}
body{{background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,sans-serif;margin:0;padding:24px 16px}}
main{{max-width:1100px;margin:auto}} h1{{font-size:24px;margin:0 0 6px}} h2{{font-size:20px;margin:44px 0 8px;border-bottom:2px solid var(--line);padding-bottom:4px}}
h3{{font-size:16px;margin:28px 0 8px}} .sub{{color:var(--ink2);font-size:13px}}
code{{font-size:12.5px;background:var(--code);padding:0 3px;border-radius:3px}} pre{{background:var(--code);padding:10px 12px;border-radius:6px;overflow-x:auto;font-size:12px;line-height:1.45}}
pre code{{background:none}}
.tldr{{border:1px solid var(--line);border-radius:8px;padding:4px 18px;background:var(--card);margin-top:18px}} .tldr h2{{border:0;margin:12px 0 0}}
.tldr li{{margin:6px 0}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0}} td,th{{border-bottom:1px solid var(--line);padding:6px 7px;text-align:left;vertical-align:top}}
th{{color:var(--ink2);font-weight:600}} table.kv td:first-child{{width:200px;color:var(--ink2)}} .nw{{white-space:nowrap}}
.wrap{{overflow-x:auto}} table.main td:nth-child(5){{min-width:280px}}
.badge{{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600;border:1px solid currentColor;white-space:nowrap}}
.badge.good{{color:var(--good)}} .badge.bad{{color:var(--bad)}} .badge.warn{{color:var(--warn)}}
figure{{margin:14px 0 26px}} .ftitle{{font-weight:600;font-size:14px;margin-bottom:4px}} figcaption{{font-size:13px;color:var(--ink2)}} .hint{{display:block;font-size:12px;opacity:.8}}
.shot img{{width:100%;border:1px solid var(--line);border-radius:6px;background:#fff}}
.pair{{display:grid;grid-template-columns:1fr 1fr;gap:14px}} @media (max-width:720px){{.pair{{grid-template-columns:1fr}} table.kv td:first-child{{width:auto}}}}
.iplot{{background:var(--card);border:1px solid var(--line);border-radius:6px;padding:4px}}
.uplot .u-legend{{font-size:12px;color:var(--ink)}} .u-axis{{color:var(--ink2)}}
</style>{UPLOT_CSS}</head><body><main>
{BODY}
</main>
<script type="application/json" id="figs">{json.dumps(FIGS, separators=(",", ":"))}</script>
{UPLOT_JS}
<script>
(function(){{
const F=JSON.parse(document.getElementById('figs').textContent);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim()||'#888';
const col=c=>c.startsWith('var(')?css(c.slice(4,-1)):c;
const plots=[];
function build(el){{
  const f=F[el.dataset.fig]; el.innerHTML='';
  const ink2=css('--ink2'), line=css('--line');
  const axis=(sc,side)=>({{scale:sc,side,stroke:ink2,grid:{{stroke:line,width:1,show:side!==1}},ticks:{{stroke:line}},label:sc==='x'?f.xl:(f.scales[sc]||''),labelSize:18,size:52}});
  const bars=f.kind==='bars'?uPlot.paths.bars({{size:[0.35,40],align:0}}):null;
  const series=[{{label:f.xl}}].concat(f.s.map((s,i)=>({{label:s.l,stroke:col(s.c),fill:f.kind==='bars'?col(s.c)+'':undefined,width:1.5,scale:s.sc,
     dash:s.d?[6,4]:undefined,paths:f.kind==='bars'?uPlot.paths.bars({{size:[0.35,40],align:i===0?-1:1}}):undefined,
     points:{{show:f.kind==='pts'||s.p,size:7}}}})));
  const scales={{x:{{time:false}}}}; Object.keys(f.scales).forEach(k=>scales[k]={{auto:true}});
  if(f.kind==='bars'){{scales.x.range=(u,a,b)=>[a-0.6,b+0.6];}}
  const axes=[axis('x',2)]; Object.keys(f.scales).forEach((k,i)=>axes.push(axis(k,i===0?3:1)));
  const opts={{width:Math.max(280,el.clientWidth-10),height:f.kind==='bars'?260:320,series,scales,axes,
    cursor:{{drag:{{x:true,y:false}}}},legend:{{live:true}}}};
  const data=[f.x].concat(f.s.map(s=>s.y));
  const u=new uPlot(opts,data,el); plots.push([u,el]);
}}
function all(){{plots.length=0;document.querySelectorAll('.iplot').forEach(build);}}
all();
let rt;window.addEventListener('resize',()=>{{clearTimeout(rt);rt=setTimeout(()=>plots.forEach(([u,el])=>u.setSize({{width:Math.max(280,el.clientWidth-10),height:u.height}})),150);}});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',all);
}})();
</script></body></html>'''

DEST.parent.mkdir(parents=True, exist_ok=True)
DEST.write_text(PAGE)
print(DEST, f'{len(PAGE)/1e6:.2f} MB', 'figs', len(FIGS))
