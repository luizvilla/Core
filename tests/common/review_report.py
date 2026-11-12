#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file review_report.py
## @brief Writes report.html: the one page to open after a campaign (procedure §8.5).
## @par Usage
##   python tests/common/review_report.py tests/out/<campaign>
## @par What the page shows, in review order
##   1. Banner: counts, new failures, known failures, findings ready to close.
##   2. One section per test case (scenario folder), failures first: purpose; conditions and
##      objectives; model snapshot and what the fixture changed (tests/l1/plant/snapshot.py);
##      results; figures from tests/l1/review_plots.json (limits dashed, thresholds dash-dot,
##      judged window shaded, driving signals below).
##      On-target test cases (a folder with target_manifest.json, L0-TIM, ADR 0005) have the same
##      four parts: purpose; conditions and objectives; target snapshot (board, MCU, flash, firmware,
##      probe) in place of the model snapshot; results; figures.
##   3. Criteria without a test-case folder (L0, block checks, BLOCKED cases).
##   4. Criteria whose verdict or measured value changed since the previous campaign.
## @return 0; a missing scenario folder or plot library is reported on the page, not fatal.
"""Reads the campaign's *.log result lines and scenario folders; never changes a verdict."""
from pathlib import Path
import csv,html,json,re,sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from make_record import parse, findings_register, finding_for, sort_key  # noqa: E402
from interactive_plot import Figure, MAX_POINTS, assets  # noqa: E402

SPECS = REPO / 'tests/l1/review_plots.json'
SCENARIOS = REPO / 'tests/l1/scenarios'

# Reference palette (dataviz skill): categorical slots in fixed order, text inks, status red.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
INK, INK2, MUTED, GRID = '#0b0b0b', '#52514e', '#8a8984', '#e4e2dc'
SURFACE, WINDOW_FILL, CRITICAL, GOOD = '#fcfcfb', '#ebe9e3', '#e34948', '#008300'


def first_number(text):
    """@brief First number in a measured-value string. @return float or None."""
    m = re.search(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', text or '')
    return float(m.group(0)) if m else None


def previous_campaign(out):
    """@brief Most recent earlier campaign that ran the same test scripts (same log names).
    @return Path or None."""
    logs = {p.name for p in out.glob('*.log')}
    others = [p for p in out.parent.iterdir()
              if p.is_dir() and p != out and re.match(r'\d{4}-\d\d-\d\d_', p.name)
              and {q.name for q in p.glob('*.log')} == logs and p.stat().st_mtime < out.stat().st_mtime]
    return max(others, key=lambda p: p.stat().st_mtime, default=None)


def resolve(value, scenario):
    """@brief Read 'acceptance:key[,key]' from the scenario JSON, or return value as is."""
    if not (isinstance(value, str) and value.startswith('acceptance:')):
        return value, 'review_plots.json'
    keys = value.split(':', 1)[1].split(',')
    acc = scenario.get('acceptance', {})
    got = [acc[k] for k in keys]
    return (got[0] if len(got) == 1 else got), f'scenario acceptance ({", ".join(keys)})'


def load_csv(path):
    """@brief control.csv as {column: list of floats}."""
    rows = list(csv.DictReader(path.open()))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


class Signals:
    """@brief Signals of a scenario folder: control.csv columns (per control tick), else
    waveforms.npz traces (electrical, full resolution), each on its own time base."""
    def __init__(self, folder):
        self.csv = load_csv(folder / 'control.csv')
        self.npz = folder / 'waveforms.npz'
        self._wave = None

    def get(self, name):
        """@return (time list in s, values list)."""
        if name in self.csv:
            return self.csv['time_s'], self.csv[name]
        if self._wave is None:
            import numpy as np
            self._wave = np.load(self.npz)
        return self._wave['time'], self._wave[name]


def plot(cid, spec, row, folder):
    """@brief Build one interactive review figure (tests/common/interactive_plot.py).
    @return (html, notes) or (None, reason)."""
    import numpy as np
    if not (folder / 'control.csv').exists():
        return None, f'no saved run in `{folder.name}/`'
    sig = Signals(folder)
    # Limits/windows named 'acceptance:…' come from the scenario as it actually ran (manifest).
    man = folder / 'manifest.json'
    scenario = json.loads(man.read_text()).get('scenario', {}) if man.exists() else {}
    limits, limit_src = resolve(spec.get('limits'), scenario)
    window, window_src = resolve(spec.get('window'), scenario)
    t_s, y = sig.get(spec['signal'])
    t_s, y = np.asarray(t_s, float), np.asarray(y, float)
    k = 1.0 if (t_s[-1] if len(t_s) else 0) > 2.0 else 1e3   # seconds for long runs, else ms
    unit_t = 's' if k == 1.0 else 'ms'
    fig = Figure(unit_t)
    notes = []
    win = [k * window[0], k * window[1]] if window else None
    events = [{'t': k * ev['t'], 'label': ev['label']} for ev in spec.get('events', [])]
    main = fig.chart(spec['unit'], 280)
    if spec.get('reference'):
        rt, rv = sig.get(spec['reference'])
        fig.line(main, spec['reference'], k * np.asarray(rt, float), rv, INK2, 1)
    ref_run = folder.parent / spec['reference_run'] if spec.get('reference_run') else None
    if ref_run and (ref_run / 'control.csv').exists():
        rt, rv = Signals(ref_run).get(spec['signal'])
        fig.line(main, f'{spec["signal"]} ({spec["reference_run"]})', k * np.asarray(rt, float), rv, INK2, 1, dash=[6, 4])
        notes.append(f'Dashed grey: the same signal in {spec["reference_run"]}/, for comparison.')
    fig.line(main, spec['signal'], k * t_s, y, SERIES[0], 2)
    for v in spec.get('thresholds', []):
        main['hlines'].append({'v': v, 'dash': [7, 3, 1, 3], 'color': MUTED})
    if spec.get('thresholds'):
        notes.append(f'Thresholds {", ".join(f"{v:g}" for v in spec["thresholds"])} {spec["unit"]} (dash-dot): '
                     'crossing points, not a band the signal must stay in.')
    if limits:
        lo, hi = limits
        main['hlines'] += [{'v': v, 'dash': [6, 4], 'color': INK2} for v in (lo, hi) if v is not None]
        inside = np.ones(len(t_s), bool) if window is None else (t_s >= window[0]) & (t_s <= window[1])
        out = inside & (((y < lo) if lo is not None else False) | ((y > hi) if hi is not None else False))
        if out.any():
            fig.line(main, 'outside limit', k * t_s, np.where(out, y, np.nan), CRITICAL, 1, points=True)
            notes.append(f'{int(out.sum())} samples outside the limit (red dots).')
        notes.append(f'Limit {lo if lo is not None else "−∞"} … {hi if hi is not None else "+∞"} '
                     f'{spec["unit"]} (dashed), from {limit_src}.')
    if window:
        notes.append(f'Judged window {k*window[0]:g}–{k*window[1]:g} {unit_t} (shaded), from {window_src}.')
    if spec.get('peak_from_manifest') and man.exists():
        ext = json.loads(man.read_text()).get('electrical_step_extrema', {})
        peak = ext.get(spec['peak_from_manifest'], {}).get('max')
        if peak is not None:
            notes.append(f'Physical peak between samples (manifest): {peak:.4g} A. The criterion judges this value.')
    if ref_run and (ref_run / 'control.csv').exists():
        rt, rv = Signals(ref_run).get(spec['signal'])
        dv = y - np.interp(t_s, np.asarray(rt, float), np.asarray(rv, float))
        c = fig.chart(f"{spec['unit']}, this − reference", 150)
        fig.line(c, f'{spec["signal"]} difference', k * t_s, dv, SERIES[0], 1.2)
        notes.append(f'Second panel: difference with {spec["reference_run"]}/ (max {np.max(np.abs(dv)):.3g} {spec["unit"]}).')
    for panel in spec.get('panels', []):
        c = fig.chart(panel['unit'], 150)
        for i, name in enumerate(panel['signals']):
            pt, pv = sig.get(name)
            fig.line(c, name, k * np.asarray(pt, float), pv, SERIES[i % len(SERIES)], 1.5)
    for c in fig.charts:
        c['window'], c['events'] = win, events
    if fig.reduced:
        notes.append(f'Long traces reduced to {MAX_POINTS:,} points, keeping each interval\'s minimum and maximum; '
                     'the raw data is in the run folder.')
    return fig.html(re.sub(r'[^A-Za-z0-9_-]', '_', cid), f'{cid}: {spec["signal"]}'), notes


MODES = {0: 'IDLE', 1: 'STARTUP', 2: 'BUSREG', 3: 'MPPT'}
SNAPSHOT = REPO / 'tests/l1/plant/snapshot.py'


def scenario_definition(meta):
    """@brief Scenario JSON of a run: the manifest copy, completed from tests/l1/scenarios by name
    for runs recorded before scenarios carried a title, purpose and criteria."""
    s = dict(meta.get('scenario', {}))
    for f in sorted(SCENARIOS.glob('*.json')):
        d = json.loads(f.read_text())
        if d.get('name') == s.get('name'):
            for k in ('title', 'purpose', 'criteria'):
                s.setdefault(k, d.get(k))
            s['_order'] = f.stem
    return s


def ensure_snapshot(folder):
    """@brief Write model.pulsim/model_power.png/model_changes.json if missing. @return error text or ''."""
    if (folder / 'model_power.png').exists() and (folder / 'model_changes.json').exists():
        return ''
    import os, subprocess
    env = {**os.environ, 'QT_QPA_PLATFORM': 'offscreen'}
    r = subprocess.run([sys.executable, str(SNAPSHOT), str(folder)], env=env, capture_output=True, text=True, timeout=300)
    return '' if r.returncode == 0 else (r.stderr.strip().splitlines() or ['snapshot failed'])[-1]


def model_parameter(model, name):
    """@brief Parameters of a named component in a saved .pulsim project (dict or None)."""
    found = []
    def walk(o):
        if isinstance(o, dict):
            if o.get('name') == name and 'parameters' in o:
                found.append(o['parameters'])
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
    walk(model)
    return found[0] if found else None


def num(v):
    """@brief Short display of a parameter value (4 significant digits for numbers)."""
    return f'{v:.4g}' if isinstance(v, float) else str(v)


def si(value, unit):
    """@brief Engineering formatting, e.g. 0.00012 F -> '120 µF'."""
    for scale, prefix in ((1e6, 'M'), (1e3, 'k'), (1, ''), (1e-3, 'm'), (1e-6, 'µ'), (1e-9, 'n')):
        if abs(value) >= scale or scale == 1e-9:
            return f'{value / scale:.4g} {prefix}{unit}'


def conditions(folder, meta, scen):
    """@brief Rows (label, html) describing what the test ran on and what it was given."""
    e = html.escape
    rows = []
    model = json.loads((folder / 'model.pulsim').read_text()) if (folder / 'model.pulsim').exists() else {}
    ch = json.loads((folder / 'model_changes.json').read_text()) if (folder / 'model_changes.json').exists() else {'changed': [], 'added': []}
    rows.append(('Plant model', f'switched, <code>full_model.pulsim</code> sha256 {e(meta.get("model_sha256", "?")[:12])}…'
                 + (' with the <b>synthetic PV panel</b>' if meta.get('pv_model_available') else '')))
    src, rs = model_parameter(model, 'Vsource'), model_parameter(model, 'Rsource')
    if meta.get('pv_model_available'):
        ref = meta.get('pv_reference', {})
        kind = 'Lambert W panel' if meta.get('pv_model') == 'lambertw' else 'diode panel'
        rows.append(('Input source', e(f'synthetic PV panel ({kind}), MPP {ref.get("Pmp_W", "?"):.3g} W at {ref.get("Vmp_V", "?"):.3g} V')
                     if ref else 'synthetic PV panel'))
    elif src:
        rows.append(('Input source', e(f'{src["waveform"].get("value")} V ideal DC') + (e(f' + {si(rs["resistance"], "Ω")}') if rs else '')))
    cbus = model_parameter(model, 'Cbus')
    if cbus:
        v0 = float(cbus.get('initial_voltage', 0) or 0)
        state = f'<b>PRECHARGED to {v0:g} V</b>' if v0 > 0 else '<b>discharged (0 V)</b>'
        rows.append(('DC bus at t = 0', f'{state}, {e(si(cbus["capacitance"], "F"))}'))
    other = [c for c in ch['changed'] if c['component'] != 'Cbus']
    if other:
        rows.append(('Other model changes', e(', '.join(f'{c["component"]} {c["parameter"]} = {c["this_run"]:.4g}' for c in other))))
    loads = []
    bleed = model_parameter(model, 'Rbus_bleed')
    if bleed:
        loads.append(f'{si(bleed["resistance"], "Ω")} bleed (board)')
    fixture = meta.get('fixture') or {}
    for a in ch['added']:
        if a['type'] == 'RESISTOR':
            when = f', switched in at {1e3 * fixture["load_step_s"]:g} ms' if a['component'] == 'DCDC_step_load' and 'load_step_s' in fixture else ', from t = 0'
            loads.append(f'{si(a["parameters"]["resistance"], "Ω")} {a["component"]}{when}')
    if loads:
        rows.append(('Loads on the bus', e('; '.join(loads))))
    grid = model_parameter(model, 'Vgrid')
    if grid and grid.get('waveform', {}).get('type') == 'sine':
        w = grid['waveform']
        rows.append(('Grid', e(f'{w["amplitude"] / 2 ** .5:.1f} V rms, {w["frequency"]:g} Hz, present from t = 0; inverter bridge and relay off')))
    cmds = []
    for q in scen.get('commands', []):
        cmds.append(f'{1e3 * q["time_s"]:g} ms: <b>{MODES.get(q["mode"], q["mode"])}</b>, v_bus_ref {q["v_bus_ref"]:g} V, '
                    f'sync {"requested" if q["sync_request"] else "off"}, VAC samples {"delivered" if q["vac_updated"] else "<b>lost</b>"}')
    if cmds:
        rows.append(('Operator commands', '<br>'.join(cmds)))
    fw = meta.get('firmware', {})
    if fw:
        rows.append(('Firmware', e(f'Core {fw.get("core_commit", "?")[:7]}{" (dirty)" if fw.get("core_dirty") else ""}, '
                                   f'profile {fw.get("profile", "?")}, inverter library {fw.get("inverter_commit", "?")[:7]}')))
    rows.append(('Simulation', e(f'{1e3 * scen.get("duration_s", 0):g} ms, step {si(scen.get("dt_s", 0), "s")}, '
                                 f'stored every {si(scen.get("store_period_s", 0), "s")}, Pulsim {meta.get("pulsim", "?")}, '
                                 f'{meta.get("wall_s", 0):.0f} s of computing')))
    return rows, [(f'{c["component"]}.{c["parameter"]}', c['accepted'], c['this_run']) for c in ch['changed']] + \
        [(f'{a["component"]} added ({a["type"].lower()})', '—', ', '.join(f'{k} {num(v)}' for k, v in a['parameters'].items() if k != 'enable_thermal_port')) for a in ch['added']]


def test_cases(out, rows):
    """@brief One entry per scenario folder of the campaign, with its criteria, conditions and snapshot."""
    cases = []
    for man in sorted(out.glob('*/manifest.json')):
        folder = man.parent
        meta = json.loads(man.read_text())
        scen = scenario_definition(meta)
        prefixes = tuple(scen.get('criteria') or [])
        mine = [r for r in rows if prefixes and r['id'].startswith(prefixes)]
        err = ensure_snapshot(folder)
        cond, changes = conditions(folder, meta, scen)
        cases.append({'folder': folder, 'title': scen.get('title') or scen.get('name', folder.name),
                      'purpose': scen.get('purpose'), 'rows': mine, 'conditions': cond, 'changes': changes,
                      'snapshot_error': err, 'order': scen.get('_order', folder.name)})
    for man in sorted(out.glob('*/target_manifest.json')):
        folder = man.parent
        meta = json.loads(man.read_text())
        prefixes = tuple(meta.get('criteria') or [])
        mine = [r for r in rows if prefixes and r['id'].startswith(prefixes)]
        cases.append({'folder': folder, 'kind': 'target', 'title': meta.get('title', folder.name),
                      'purpose': meta.get('purpose'), 'rows': mine,
                      'conditions': [(k, html.escape(v)) for k, v in meta.get('conditions', [])],
                      'target': meta.get('target', []), 'report': folder / meta.get('report', 'timing_report.json'),
                      'figures': meta.get('figures', 'tim01'),
                      'order': 'zz_' + folder.name})
    # A criterion judged on several runs (the PV 25/50 ns pair) is listed under each of them.
    return cases


# L0-TIM-01 figure: each section stacked by its mean; one hue per group in fixed categorical order (dataviz palette,
# slots 1-5), its sections in shades of that hue (base, then lighter), so a group reads as one colour and its parts
# stay apart. Labels: (section key, legend text).
TIMING_GROUPS = [('boost', '#2a78d6', [('boost_cold', 'setters + cold loop'), ('boost_hot', 'hot loop')]),
                 ('inverter logic', '#eb6834', [('inv_stage', 'stage logic'), ('outputs', 'outputs, relay')]),
                 ('calculateDuty', '#1baf7a', [('duty_pll', 'PLL'), ('duty_sogi', 'SOGI V+I'), ('duty_park', 'Park V+I'),
                                               ('duty_syncwin', 'sync window'), ('duty_mode_ff', 'mode + feedforward'),
                                               ('duty_current_pi', 'current PI'), ('duty_output', 'output + duty')]),
                 ('grid observer', '#eda100', [('observer', 'observer, quality gate')]),
                 ('scope', '#e87ba4', [('scope', 'snapshot + acquire')])]


def shade(hex_color, k, n=2):
    """@brief Shade k of n of a hue. Two shades: the base, then 45 % towards white. More: an even ramp from 35 %
    towards black to 65 % towards white, so neighbours stay apart."""
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    m = (0.0, 0.45)[min(k, 1)] if n <= 2 else -0.35 + 1.0 * k / (n - 1)
    f = (lambda c: c * (1 + m)) if m < 0 else (lambda c: c + (255 - c) * m)
    return '#%02x%02x%02x' % tuple(round(f(c)) for c in (r, g, b))


# Short names for segments too narrow for the legend text.
TIMING_SHORT = {'boost_cold': 'cold', 'boost_hot': 'hot', 'inv_stage': 'stage', 'outputs': 'out', 'duty_pll': 'PLL',
                'duty_sogi': 'SOGI', 'duty_park': 'Park', 'duty_syncwin': 'win', 'duty_mode_ff': 'mode+ff',
                'duty_current_pi': 'PI', 'duty_output': 'output', 'observer': 'obs', 'scope': 'scope', 'inv_duty': 'duty'}


def text_on(fill):
    """@brief Text ink with contrast on a fill: white on dark fills, the primary ink otherwise."""
    r, g, b = (int(fill[i:i + 2], 16) / 255 for i in (1, 3, 5))
    lum = lambda c: c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    return '#ffffff' if 0.2126 * lum(r) + 0.7152 * lum(g) + 0.0722 * lum(b) < 0.18 else INK


def timing_groups(rows, bucket):
    """@brief TIMING_GROUPS as present in a report: a report without the library probes (layout 1) shows
    calculateDuty as one section."""
    if f'{bucket}/duty_pll' in rows:
        return TIMING_GROUPS
    return [g if g[0] != 'calculateDuty' else (g[0], g[1], [('inv_duty', 'PLL, SOGI, current loop')]) for g in TIMING_GROUPS]


VSI_NAMES = ['OFF', 'VOLTAGE_SYNC', 'GRID_CONNECT', 'GRID_CONNECTED', 'RUN', 'STOPPING', 'DISCONNECT', 'FAULT',
             'PWM_SETTLE', 'PWM_WAIT']


def event_text(sec):
    """@brief 'ev:vsi 8>2' -> 'vsi PWM_SETTLE → GRID_CONNECT'."""
    t = sec[3:]
    if t.startswith('vsi '):
        a, b = t[4:].split('>')
        return f'vsi {VSI_NAMES[int(a)]} → {VSI_NAMES[int(b)]}'
    return t


def tim02_tables(rep, budget_us=80.0):
    """@brief L0-TIM-02 table views: every section per bucket (as L0-TIM-01, plus the tick parts), then every event."""
    e = html.escape
    hz = rep['core_hz']
    rows = {r['name']: r for r in rep['rows']}
    u = lambda c: c * 1e6 / hz
    mean = lambda r: u(r['sum_cycles'] / max(r['calls'], 1))
    p = [timing_table(rep, TIM02_COLS, 'tim02_vsi_seen')]
    p.append('<div class="wrap"><table><tr><th>Bucket</th><th>Event</th><th>Ticks</th><th>Worst tick</th><th>Mean tick</th></tr>')
    for n, r in rows.items():
        if '/ev:' not in n:
            continue
        b, sec = n.split('/', 1)
        flag = ' <b>over</b>' if u(r['max_cycles']) > budget_us else ''
        p.append(f'<tr><td>{e(b)}</td><td>{e(event_text(sec))}</td><td>{r["calls"]}</td><td>{u(r["max_cycles"]):.1f}{flag}</td><td>{mean(r):.1f}</td></tr>')
    p.append('</table></div>')
    return '\n'.join(p)


def tim02_event_figure(rep, budget_us=80.0, period_us=100.0):
    """@brief Inline SVG: one bar per (bucket, event), the worst whole tick it happened on; over-budget bars in the
    status red with the word "over"."""
    e = html.escape
    hz = rep['core_hz']
    u = lambda c: c * 1e6 / hz
    evs = [(r['name'].split('/', 1)[0], event_text(r['name'].split('/', 1)[1]), u(r['max_cycles']), r['calls'])
           for r in rep['rows'] if '/ev:' in r['name']]
    if not evs:
        return '<p class="sub">No event recorded.</p>'
    W, left, right, top, bar, gap = 960, 330, 90, 24, 16, 4
    xmax = max(period_us, 10 * (int(max(v for _, _, v, _ in evs) / 10) + 2))
    order = [b['name'] for b in rep['buckets']]
    evs.sort(key=lambda t: (order.index(t[0]), -t[2]))
    H = top + len(evs) * (bar + gap) + 30
    x = lambda v: left + (W - left - right) * min(v, xmax) / xmax
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" role="img" aria-label="Worst tick per event" '
           f'style="font:12px system-ui,sans-serif;background:{SURFACE}">']
    for v in range(0, int(xmax) + 1, 10):
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{top - 4}" y2="{H - 26}" stroke="{GRID}"/>'
                   f'<text x="{x(v):.1f}" y="{H - 10}" fill="{MUTED}" text-anchor="middle">{v} µs</text>')
    for v, label in ((budget_us, f'budget {budget_us:g} µs'), (period_us, f'period {period_us:g} µs')):
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{top - 8}" y2="{H - 26}" stroke="{INK2}" stroke-dasharray="6 4"/>'
                   f'<text x="{x(v) - 4:.1f}" y="{top - 12}" fill="{INK2}" text-anchor="end">{e(label)}</text>')
    prev = None
    for i, (b, text, v, n) in enumerate(evs):
        y = top + i * (bar + gap)
        if b != prev and i:
            svg.append(f'<line x1="0" x2="{W}" y1="{y - gap / 2:.1f}" y2="{y - gap / 2:.1f}" stroke="{GRID}"/>')
        over = v > budget_us
        col = CRITICAL if over else SERIES[0]
        svg.append(f'<text x="{left - 8}" y="{y + bar / 2 + 4}" fill="{INK}" text-anchor="end">'
                   f'<tspan fill="{INK2}">{e(b) if b != prev else ""}</tspan>  {e(text)}</text>'
                   f'<rect x="{left}" y="{y}" width="{max(x(v) - left, 1):.1f}" height="{bar}" rx="2" fill="{col}">'
                   f'<title>{e(b)} · {e(text)}: worst tick {v:.1f} µs over {n} occurrence(s)</title></rect>'
                   f'<text x="{x(v) + 6:.1f}" y="{y + bar / 2 + 4}" fill="{INK}">{v:.1f}{" over" if over else ""}</text>')
        prev = b
    svg.append('</svg>')
    return ''.join(svg)


# L0-TIM-02 adds the shield I/O around controlStep: two more groups (dataviz slots 6 and 7; the 7-group order
# sensors, boost, inverter logic, calculateDuty, observer, writes, scope passes the palette validator).
SENSOR_GROUP = ('sensor reads', '#008300', [('tick_sensors', 'sensor reads')])
WRITE_GROUP = ('PWM + relay writes', '#6250d6', [('tick_apply', 'PWM + relay writes')])
# What the probes do not cover (calls, struct copies, the task's own load bookkeeping): neutral grey, last in the bar,
# so every bar adds up to its measured total (TIM-01: controlStep + scope; TIM-02: the whole tick).
REST_GROUP = ('not in a section', '#b9b7b0', [('_rest', 'calls + timing bookkeeping')])
TIMING_SHORT.update({'tick_sensors': 'sensors', 'tick_apply': 'writes', 'tick_scope': 'scope', '_rest': 'rest',
                     'tick_control': 'controlStep'})


def series_groups(rows, bucket, kind):
    """@brief Groups of one test, with the row keys it reports. L0-TIM-01: the controlStep sections and the scope.
    L0-TIM-02: sensor reads, the same sections measured in the critical tick ('2:' rows), PWM and relay writes,
    scope, in the order the tick runs them. Shared groups keep their hue and shades in both tests."""
    base = timing_groups(rows, bucket)
    if kind == 'tim01':
        return base + [REST_GROUP]
    out = [SENSOR_GROUP]
    if f'{bucket}/2:boost_cold' in rows:
        for gname, hue, secs in base:
            if gname != 'scope':
                out.append((gname, hue, [('2:' + k, t) for k, t in secs]))
    else:  # a run recorded before TIM-02 kept the sections: controlStep as one block, never dropped
        out.append(('controlStep', '#1baf7a', [('tick_control', 'controlStep (not split in this run)')]))
    out += [WRITE_GROUP, ('scope', '#e87ba4', [('tick_scope', 'snapshot + acquire')]), REST_GROUP]
    return out


def bar_rows(rep, kind, tag=''):
    """@brief One bar per bucket: segments (group, hue, shade index, shade count, key, text, mean µs, worst µs) and
    the worst total (L0-TIM-01: worst controlStep + worst scope; L0-TIM-02: worst whole tick)."""
    hz = rep['core_hz']
    rows = {r['name']: r for r in rep['rows']}
    u = lambda c: c * 1e6 / hz
    out = []
    for b in [x['name'] for x in rep['buckets']]:
        mean_of = lambda k: u(rows[f'{b}/{k}']['sum_cycles'] / max(rows[f'{b}/{k}']['calls'], 1))
        if kind == 'tim01':
            total = mean_of('controlStep') + mean_of('scope')
            worst = u(rows[f'{b}/controlStep']['max_cycles'] + rows[f'{b}/scope']['max_cycles'])
        else:
            total = mean_of('tick')
            worst = u(rows[f'{b}/tick']['max_cycles'])
        segs = []
        for gname, hue, secs in series_groups(rows, b, kind):
            for k, (key, text) in enumerate(secs):
                if key == '_rest':
                    rest = total - sum(x[6] for x in segs)
                    if rest > 0.05:
                        segs.append((gname, hue, k, len(secs), key, text, rest, rest))
                    continue
                r = rows.get(f'{b}/{key}')
                if r and r['calls']:
                    segs.append((gname, hue, k, len(secs), key, text, u(r['sum_cycles'] / r['calls']), u(r['max_cycles'])))
        out.append({'bucket': b, 'tag': tag, 'segs': segs, 'worst': worst, 'mean': total})
    return out


def timing_xmax(rep, period_us=100.0):
    """@brief Common time axis of every timing figure of a report, so TIM-01 and TIM-02 read on the same scale."""
    worst = max([r['worst'] for r in bar_rows(rep, 'tim01')] +
                ([r['worst'] for r in bar_rows(rep, 'tim02')] if any(n['name'].endswith('/tick') for n in rep['rows']) else []))
    return max(period_us, 10 * (int((worst + 6) / 10) + 1))


def legend_svg(groups, W=960):
    """@brief Legend, fixed width: one cluster per group (name, a swatch per section), wrapped, then the worst mark."""
    e = html.escape
    legend, lx, ly = [], 0, 16
    for gname, hue, secs in groups:
        width = 8 + 7 * len(gname) + sum(22 + 6.3 * len(t) for _, t in secs)
        if lx and lx + width > W - 10:
            lx, ly = 0, ly + 20
        legend.append(f'<text x="{lx}" y="{ly}" fill="{INK}" font-weight="600">{e(gname)}</text>')
        sx = lx + 8 + 7 * len(gname)
        for k, (_, text) in enumerate(secs):
            legend.append(f'<rect x="{sx:.0f}" y="{ly - 10}" width="12" height="12" rx="2" fill="{shade(hue, k, len(secs))}"/>'
                          f'<text x="{sx + 16:.0f}" y="{ly}" fill="{INK2}">{e(text)}</text>')
            sx += 22 + 6.3 * len(text)
        lx += width + 18
    if lx + 200 > W - 10:
        lx, ly = 0, ly + 20
    legend.append(f'<line x1="{lx + 4}" x2="{lx + 4}" y1="{ly - 12}" y2="{ly + 3}" stroke="{INK}" stroke-width="2"/>'
                  f'<text x="{lx + 10}" y="{ly}" fill="{INK2}">worst case</text>')
    return (f'<svg viewBox="0 0 {W} {ly + 10}" width="100%" style="font:12px system-ui,sans-serif;background:{SURFACE}">'
            + ''.join(legend) + '</svg>')


def _layout(bars):
    """@brief y of each bar; paired bars (a tag) sit close, buckets apart. @return (ys, bar height, total height)."""
    paired = any(b['tag'] for b in bars)
    bar, inner, outer, top = (20, 3, 14, 30) if paired else (24, 10, 10, 30)
    ys, y, prev = [], top, None
    for b in bars:
        if prev is not None:
            y += bar + (inner if paired and b['bucket'] == prev else outer)
        ys.append(y)
        prev = b['bucket']
    return ys, bar, ys[-1] + bar + 30


def _segment_label(width, text, short, mean, size_full=11):
    """@brief Direct label of a segment: name and value, short name and value, value, or nothing."""
    full, sh = f'{text} {mean:.1f}', f'{short} {mean:.1f}'
    if width >= 6.3 * len(full) + 8:
        return full, size_full
    if width >= 6.3 * len(sh) + 6:
        return sh, size_full
    if width >= 22:
        return f'{mean:.1f}', 11
    if width >= 14:
        return f'{mean:.1f}', 9
    return '', 0


def _bars_svg(bars, x, ys, bar, H, width, xmax, budget_us, period_us, label_mode, worst_text):
    """@brief Plot body shared by the overview and the detailed view (no bucket labels)."""
    e = html.escape
    svg = [f'<svg width="{width}" height="{H}" viewBox="0 0 {width} {H}" role="img" aria-label="Mean cost per section and worst case" '
           f'style="font:12px system-ui,sans-serif;background:{SURFACE};flex:none">']
    step = 1 if label_mode == 'detail' else 10
    for v in range(0, int(xmax) + 1, step):
        major = v % (5 if label_mode == 'detail' else 10) == 0
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="24" y2="{H - 26}" stroke="{GRID}" stroke-width="{1 if major else 0.5}"'
                   f'{"" if major else " stroke-dasharray=\"2 3\""}/>')
        if major:
            svg.append(f'<text x="{x(v):.1f}" y="{H - 10}" fill="{MUTED}" text-anchor="middle">{v} µs</text>')
    for v, label in ((budget_us, f'budget {budget_us:g} µs'), (period_us, f'period {period_us:g} µs')):
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="20" y2="{H - 26}" stroke="{INK2}" stroke-dasharray="6 4"/>'
                   f'<text x="{x(v) - 4:.1f}" y="16" fill="{INK2}" text-anchor="end">{e(label)}</text>')
    for b, y in zip(bars, ys):
        acc = 0.0
        for gname, hue, k, n, key, text, mean, worst in b['segs']:
            fill = shade(hue, k, n)
            x0, x1 = x(acc), x(acc + mean)
            svg.append(f'<rect x="{x0:.1f}" y="{y}" width="{max(x1 - x0, 1):.1f}" height="{bar}" fill="{fill}" stroke="{SURFACE}" '
                       f'stroke-width="2"><title>{e(b["bucket"])} {e(b["tag"])} · {e(gname)} · {e(text)}: mean {mean:.2f} µs, '
                       + (f'the measured total minus the probed sections' if key == '_rest' else f'worst {worst:.2f} µs')
                       + '</title></rect>')
            if label_mode == 'detail':
                lbl, size = _segment_label(x1 - x0, text, TIMING_SHORT.get(key.split(':')[-1], key), mean)
            else:
                lbl, size = (f'{mean:.1f}', 11) if x1 - x0 >= 26 else ('', 0)
            if lbl:
                svg.append(f'<text x="{(x0 + x1) / 2:.1f}" y="{y + bar / 2 + 4}" fill="{text_on(fill)}" font-size="{size}" '
                           f'text-anchor="middle" pointer-events="none">{e(lbl)}</text>')
            acc += mean
        svg.append(f'<line x1="{x(b["worst"]):.1f}" x2="{x(b["worst"]):.1f}" y1="{y - 2}" y2="{y + bar + 2}" stroke="{INK}" stroke-width="2">'
                   f'<title>{e(b["bucket"])} {e(b["tag"])}: worst {b["worst"]:.1f} µs</title></line>'
                   f'<text x="{x(b["worst"]) + 6:.1f}" y="{y + bar / 2 + 4}" fill="{INK}">{e(worst_text(b))}</text>')
    svg.append('</svg>')
    return ''.join(svg)


def _labels_svg(bars, ys, bar, H, label_w, sticky):
    e = html.escape
    style = f'font:12px system-ui,sans-serif;flex:none;background:{SURFACE}' + (';position:sticky;left:0;z-index:1' if sticky else '')
    out = [f'<svg width="{label_w}" height="{H}" style="{style}">']
    prev = None
    for b, y in zip(bars, ys):
        name = b['bucket'] if b['bucket'] != prev else ''
        tag = f' <tspan fill="{INK2}" font-size="11">{e(b["tag"])}</tspan>' if b['tag'] else ''
        out.append(f'<text x="{label_w - 8}" y="{y + bar / 2 + 4}" fill="{INK}" text-anchor="end">{e(name)}{tag}</text>')
        prev = b['bucket']
    out.append('</svg>')
    return ''.join(out)


def timing_overview(bars, groups, xmax, budget_us=80.0, period_us=100.0):
    """@brief Overview, fitted to the page: every bar on one screen, values where they fit."""
    ys, bar, H = _layout(bars)
    label_w = 170 if any(b['tag'] for b in bars) else 130
    W, right = 960, 60
    x = lambda v: label_w + 8 + (W - label_w - 8 - right) * min(v, xmax) / xmax
    body = _bars_svg(bars, x, ys, bar, H, W, xmax, budget_us, period_us, 'overview', lambda b: f'{b["worst"]:.1f}')
    labels = _labels_svg(bars, ys, bar, H, label_w, False)
    labels = labels[labels.index('>') + 1:-len('</svg>')]
    body = body.replace(f'<svg width="{W}" height="{H}"', '<svg width="100%"', 1)
    return legend_svg(groups) + body[:-len('</svg>')] + labels + '</svg>'


def timing_detail(bars, groups, xmax, budget_us=80.0, period_us=100.0, px_per_us=40):
    """@brief Detailed view at px_per_us: every segment labelled (name and mean); scrolls, bucket names pinned."""
    ys, bar, H = _layout(bars)
    label_w = 170 if any(b['tag'] for b in bars) else 130
    plot_w = int(xmax * px_per_us) + 8 + 180
    x = lambda v: 8 + px_per_us * min(v, xmax)
    body = _bars_svg(bars, x, ys, bar, H, plot_w, xmax, budget_us, period_us, 'detail',
                     lambda b: f'worst {b["worst"]:.1f} µs (mean {b["mean"]:.1f})')
    return (legend_svg(groups) + f'<div class="wrap" style="display:flex;overflow-x:auto;border:1px solid {GRID};border-radius:6px">'
            + _labels_svg(bars, ys, bar, H, label_w, True) + body + '</div>')


def paired_bars(rep):
    """@brief Per bucket, the L0-TIM-01 bar then the L0-TIM-02 bar."""
    a, b = bar_rows(rep, 'tim01', 'TIM-01'), bar_rows(rep, 'tim02', 'TIM-02')
    return [x for pair in zip(a, b) for x in pair]


def timing_table(rep, cols, states_key='vsi_seen'):
    """@brief Table view: worst case (mean) in µs per bucket for the given (row key, header) columns."""
    e = html.escape
    hz = rep['core_hz']
    rows = {r['name']: r for r in rep['rows']}
    u = lambda c: c * 1e6 / hz
    p = ['<div class="wrap"><table><tr><th>Bucket</th>' + ''.join(f'<th>{e(h)}</th>' for _, h in cols) + '</tr>']
    for bk in rep['buckets']:
        b = bk['name']
        cells = []
        for key, _ in cols:
            r = rows.get(f'{b}/{key}')
            cells.append('<td>—</td>' if not r or not r['calls'] else
                         f"<td>{u(r['max_cycles']):.1f} <span class=\"sub\">({u(r['sum_cycles'] / r['calls']):.1f})</span></td>")
        p.append(f'<tr><td>{e(b)}</td>' + ''.join(cells) + '</tr>')
    p.append('</table></div><p class="sub">Worst case, mean in brackets, in µs. Inverter states reached: '
             + '; '.join(f"{e(b['name'])} {e(', '.join(VSI_NAMES[m] for m in b.get(states_key, [])))}" for b in rep['buckets']) + '.</p>')
    return '\n'.join(p)


TIM01_COLS = [('boost_cold', 'boost_cold'), ('boost_hot', 'boost_hot'), ('inv_stage', 'inv_stage'), ('inv_duty', 'inv_duty'),
              ('duty_pll', 'duty_pll'), ('duty_sogi', 'duty_sogi'), ('duty_park', 'duty_park'), ('duty_syncwin', 'duty_syncwin'),
              ('duty_mode_ff', 'duty_mode_ff'), ('duty_current_pi', 'duty_current_pi'), ('duty_output', 'duty_output'),
              ('observer', 'observer'), ('outputs', 'outputs'), ('controlStep', 'controlStep'), ('scope', 'scope')]
TIM02_COLS = ([('tick', 'whole tick'), ('tick_sensors', 'sensors')] + [('2:' + k, h) for k, h in TIM01_COLS if k not in ('controlStep', 'scope')]
              + [('tick_control', 'controlStep'), ('tick_apply', 'writes'), ('tick_scope', 'scope')])


def verdict_badge(r):
    """@brief HTML badge: verdict plus finding/new tag, never color alone."""
    v = r['verdict']
    tag = f' (known {r["finding"]["issue"]})' if r.get('finding') else (' NEW' if v == 'FAIL' else '')
    cls = {'PASS': 'pass', 'FAIL': 'fail', 'INFO': 'info', 'BLOCKED': 'blocked'}.get(v, 'info')
    icon = {'PASS': '✓', 'FAIL': '✗', 'INFO': 'i', 'BLOCKED': '–'}.get(v, '')
    return f'<span class="badge {cls}{" new" if tag == " NEW" else ""}">{icon} {v}{html.escape(tag)}</span>'



def page_head(title):
    """@brief Shared <head> and styles of the review pages."""
    return f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title><style>
:root{{color-scheme:light;--bg:{SURFACE};--ink:{INK};--ink2:{INK2};--line:{GRID};--bad:{CRITICAL};--good:{GOOD}}}
body{{background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif;margin:0;padding:24px 16px;}}
main{{max-width:1000px;margin:auto}} h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:19px;margin:40px 0 4px;border-bottom:2px solid var(--line);padding-bottom:4px}}
h3{{font-size:15px;margin:20px 0 6px;color:var(--ink2);text-transform:uppercase;letter-spacing:.04em}}
.sub{{color:var(--ink2);font-size:13px}} .banner{{border:1px solid var(--line);border-radius:8px;padding:12px 16px;margin:16px 0}}
.banner.bad{{border-color:var(--bad)}} .counts span{{margin-right:18px;font-weight:600}} .purpose{{margin:6px 0 4px}}
.badge{{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600;border:1px solid var(--line);white-space:nowrap}}
.badge.pass{{color:var(--good)}} .badge.fail{{color:var(--bad)}} .badge.new{{background:var(--bad);color:#fff;border-color:var(--bad)}}
.badge.info,.badge.blocked{{color:var(--ink2)}} figure{{margin:12px 0 24px}} figure img{{max-width:100%;border:1px solid var(--line);border-radius:6px;background:#fff}}
figcaption{{font-size:13px;color:var(--ink2)}} table{{border-collapse:collapse;width:100%;font-size:13px;margin:4px 0}}
td,th{{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left;vertical-align:top}} th{{color:var(--ink2);font-weight:600}}
table.kv td:first-child{{width:170px;color:var(--ink2)}} .check li{{margin:4px 0}} code{{font-size:12px}}
details summary{{cursor:pointer;color:var(--ink2)}} .wrap{{overflow-x:auto}} nav a{{margin-right:12px;font-size:13px}}
</style></head><body><main>
'''


def snapshot_html(f, error, changes):
    """@brief Model snapshot part: pictures, the exact project, and the changes from the accepted model."""
    e = html.escape
    p = []
    if (f / 'model_power.png').exists():
        p.append(f'<figure><a href="{e(f.name)}/model_power.png"><img alt="Power stage of the model this test ran on" src="{e(f.name)}/model_power.png"></a>'
                 f'<figcaption>Power stage as run (click for full size). Everything, including the firmware block: '
                 f'<a href="{e(f.name)}/model_full.png">model_full.png</a>. Open it in PulsimGUI: '
                 f'<code>tests/l1/open_gui.sh --results {e(str(f.relative_to(REPO)) if f.is_relative_to(REPO) else str(f))}</code> '
                 f'(the exact project is <code>{e(f.name)}/model.pulsim</code>).</figcaption></figure>')
    else:
        p.append(f'<p class="sub">No snapshot: {e(error or "not generated")}</p>')
    if changes:
        p.append('<table><tr><th>Changed from the accepted model</th><th>Accepted</th><th>This test</th></tr>')
        for ch in changes:
            p.append(f'<tr><td>{e(ch[0])}</td><td>{e(num(ch[1]))}</td><td><b>{e(num(ch[2]))}</b></td></tr>')
        p.append('</table>')
    return '\n'.join(p)


def key_numbers(folder):
    """@brief Headline values of an exploratory run, from its control log and traces. @return list of (label, text)."""
    import numpy as np
    sig = Signals(folder)
    c = sig.csv
    t = np.asarray(c['time_s'])
    out = []
    vb = np.asarray(c['VDCBus'])
    out.append(('DC bus (sensed)', f'start {vb[0]:.1f} V, min {vb.min():.1f} V, max {vb.max():.1f} V, end {vb[-1]:.1f} V'))
    def first(name):
        v = np.asarray(c[name]); i = np.flatnonzero(v > 0.5)
        return f'{t[i[0]]:.3f} s' if len(i) else 'never'
    out.append(('Grid', f'qualified at {first("grid_qualified")}, synchronized at {first("synchronized")}, '
                        f'observer trip at {first("observer_trip")}'))
    modes = [MODES.get(int(m), m) for m in c['boost_mode']]
    seq = [m for i, m in enumerate(modes) if i == 0 or m != modes[i - 1]]
    out.append(('Boost modes', ' → '.join(seq)))
    if sig.npz.exists():
        w = np.load(sig.npz)
        if 'pv_power' in w.files:
            tt = w['time']; last = tt >= tt[-1] - 0.1 * (tt[-1] - tt[0])
            out.append(('PV power, last 10 % of the run', f'{np.mean(w["pv_power"][last]):.1f} W of {np.mean(w["pv_mpp_power"][last]):.1f} W available'))
    return out


def study_page(out):
    """@brief Review page of a study (tests/l1/study.py): summary of every case, then the four parts of each finished one."""
    e = html.escape
    st = json.loads((out / 'status.json').read_text())
    study = st['study']
    p = [page_head('Study review'),
         f'<h1>{e(study.get("title", study["name"]))}</h1><div class="sub">{e(out.name)} · started {e(st["started"])} · '
         f'{st["jobs"]} parallel jobs{" · finished " + e(st["finished"]) if st.get("finished") else " · still running"}</div>',
         f'<p class="purpose">{e(study.get("purpose", ""))}</p>',
         '<p class="sub">Exploratory runs: no PASS/FAIL. What deserves a verdict becomes a catalogue criterion later, with its limits decided then.</p>',
         '<h3>Summary</h3><div class="wrap"><table><tr><th>Case</th><th>State</th><th>Simulated</th><th>Computing</th><th>Headline</th></tr>']
    sections = []
    for case in st['cases']:
        f = out / case['folder']
        state = case['state']
        cost = (f'{case["wall_s"] / 60:.0f} min' if 'wall_s' in case else f'~{case["estimate_s"] / 60:.0f} min estimated'
                + (f', since {case["started"][11:16]}' if state == 'running' else ''))
        head = ''
        if state == 'done' and (f / 'manifest.json').exists():
            try:
                nums = key_numbers(f)
                head = e(nums[0][1])
            except Exception as ex:
                nums, head = [], e(f'no headline: {ex}')
        elif state == 'failed':
            head = e(case.get('error', ''))
        badge = {'done': 'pass', 'failed': 'fail'}.get(state, 'info')
        p.append(f'<tr><td><a href="#{e(case["folder"])}">{e(case["title"])}</a></td><td><span class="badge {badge}">{e(state)}</span></td>'
                 f'<td>{case["simulated_s"]:g} s</td><td>{e(cost)}</td><td>{head}</td></tr>')
        sc = json.loads((f / 'scenario.json').read_text())
        sec = [f'<h2 id="{e(case["folder"])}">{e(case["title"])} <span class="sub">({e(case["folder"])}/ · {e(state)})</span></h2>',
               f'<p class="purpose">{e(sc.get("purpose", ""))}</p>']
        if state != 'done':
            sec.append(f'<p class="sub">{"Running" if state == "running" else "Not run yet" if state == "queued" else "Failed: " + e(case.get("error", ""))}. '
                       f'Log: <code>{e(case["folder"])}/run.log</code>.</p>')
            sections.append(sec)
            continue
        meta = json.loads((f / 'manifest.json').read_text())
        err = ensure_snapshot(f)
        cond, changes = conditions(f, meta, {**sc, **meta.get('scenario', {})})
        if sc.get('grid'):
            cond.insert(4, ('Grid profile', '<br>'.join(e(f'from {g["t_s"]:g} s: {g["v_rms"]:g} V rms, {g["f_hz"]:g} Hz') for g in sc['grid'])))
        if sc.get('light'):
            cond.insert(2, ('Light profile', e(', '.join(f'{100 * s:g} % at {t:g} s' for t, s in sc['light']))))
        if sc.get('open_loop_duty') is not None:
            cond.insert(1, ('Open loop', f'<b>both legs held at duty {sc["open_loop_duty"]:g}</b>; the firmware runs but is not obeyed'))
        sec.append('<h3>1 · Test conditions and objectives</h3><div class="wrap"><table class="kv">')
        sec += [f'<tr><td>{e(k)}</td><td>{v}</td></tr>' for k, v in cond]
        sec.append('<tr><td>Objective</td><td>Exploratory: observe, no verdict</td></tr></table></div>')
        sec.append('<h3>2 · Model snapshot</h3>')
        sec.append(snapshot_html(f, err, changes))
        sec.append('<h3>3 · Results</h3><table class="kv">')
        sec += [f'<tr><td>{e(k)}</td><td>{e(v)}</td></tr>' for k, v in nums]
        sec.append('</table><h3>4 · Figures</h3>')
        specs = json.loads((f / 'figures.json').read_text()) if (f / 'figures.json').exists() else []
        if not specs:
            sec.append('<p class="sub">No figure declared for this case in the study file.</p>')
        for i, spec in enumerate(specs):
            try:
                fig_html, notes = plot(f'{case["folder"]}-{i}', spec, {}, f)
            except Exception as ex:
                fig_html, notes = None, f'plot failed: {ex}'
            sec.append(f'<figure><div><b>{e(spec.get("title", spec["signal"]))}</b></div>')
            sec.append(f'{fig_html}<figcaption>{" ".join(e(n) for n in notes)}</figcaption>'
                       if fig_html else f'<figcaption>No plot: {e(str(notes))}</figcaption>')
            sec.append('</figure>')
        sections.append(sec)
    p.append('</table></div>')
    for sec in sections:
        p += sec
    p.append(assets() + '</main></body></html>')
    (out / 'report.html').write_text('\n'.join(p), encoding='utf-8')
    print(f'Review: {out / "report.html"}')
    return 0


def main(out):
    out = Path(out)
    if (out / 'status.json').exists():
        return study_page(out)
    rows = parse(sorted(out.glob('*.log')))
    findings = findings_register()
    for r in rows:
        r['finding'] = finding_for(r['id'], findings) if r['verdict'] == 'FAIL' else None
    count = {v: sum(r['verdict'] == v for r in rows) for v in ('PASS', 'FAIL', 'INFO', 'BLOCKED')}
    new = [r for r in rows if r['verdict'] == 'FAIL' and not r['finding']]
    known = [r for r in rows if r['finding']]
    closable = [f for f in findings if f['open'] and any(r['id'].startswith(tuple(f['criteria'])) for r in rows)
                and not any(r['verdict'] in ('FAIL', 'BLOCKED') for r in rows if r['id'].startswith(tuple(f['criteria'])))]
    specs = {k: v for k, v in json.loads(SPECS.read_text()).items() if not k.startswith('_')} if SPECS.exists() else {}

    prev = previous_campaign(out)
    changes = []
    if prev:
        before = {r['id']: r for r in parse(sorted(prev.glob('*.log')))}
        for r in rows:
            b = before.get(r['id'])
            if b is None:
                changes.append((r, None, 'new criterion'))
            elif b['verdict'] != r['verdict']:
                changes.append((r, b, f'{b["verdict"]} → {r["verdict"]}'))
            else:
                x, y = first_number(b['measured']), first_number(r['measured'])
                if x is not None and y is not None and abs(y - x) > 0.01 * max(abs(x), 1e-9):
                    changes.append((r, b, 'measured value moved by more than 1 %'))
        for cid in before:
            if cid not in {r['id'] for r in rows}:
                changes.append(({'id': cid, 'verdict': '—', 'desc': before[cid]['desc'], 'measured': '', 'finding': None},
                                before[cid], 'not run in this campaign'))

    e = html.escape
    cases = test_cases(out, rows)
    claimed = {r['id'] for c in cases for r in c['rows']}
    figures_for = {}
    for c in cases:
        figs = []
        for r in sorted([r for r in c['rows'] if r['id'] in specs], key=lambda r: (r['verdict'] != 'FAIL', r['id'])):
            spec = specs[r['id']]
            try:
                fig_html, notes = plot(r['id'], spec, r, c['folder'])
            except Exception as ex:  # report, never hide, a plotting problem
                fig_html, notes = None, f'plot failed: {ex}'
            figs.append((r, fig_html, notes))
        figures_for[c['folder'].name] = figs
    # On-target cases stay together in test order (TIM-01 before TIM-02, so a section reads before/after);
    # the block goes first when any of them fails. Other cases: failures first.
    target_fail = any(r['verdict'] == 'FAIL' for c in cases if c.get('kind') == 'target' for r in c['rows'])
    cases.sort(key=lambda c: ((not target_fail, 0, c['folder'].name) if c.get('kind') == 'target' else
                              (not any(r['verdict'] == 'FAIL' for r in c['rows']), 1, c['order'])))

    p = []
    p.append(page_head('Campaign review') + f'''<h1>Campaign review</h1><div class="sub">{e(out.name)}{f" · compared with {e(prev.name)}" if prev else " · no earlier campaign to compare"}</div>''')
    bad = bool(new) or not rows
    p.append(f'<div class="banner{" bad" if bad else ""}"><div class="counts">'
             f'<span>{count["PASS"]} PASS</span><span>{count["FAIL"]} FAIL</span><span>{count["INFO"]} INFO</span><span>{count["BLOCKED"]} BLOCKED</span></div>')
    p.append('<div><b>New failures:</b> ' + (', '.join(e(r['id']) for r in new) if new else 'none') + '</div>')
    p.append('<div><b>Known failures:</b> ' + (', '.join(f'{e(r["id"])} ({e(r["finding"]["issue"])})' for r in known) if known else 'none') + '</div>')
    if closable:
        p.append('<div><b>Ready to close:</b> ' + ', '.join(e(f['issue']) for f in closable) + '</div>')
    p.append('</div>')
    if cases:
        p.append('<nav><b>Test cases:</b> ' + ' '.join(
            f'<a href="#{e(c["folder"].name)}">{"✗ " if any(r["verdict"] == "FAIL" for r in c["rows"]) else ""}{e(c["title"])}</a>' for c in cases) + '</nav>')
    p.append('''<h3>How to review (procedure §8.5)</h3><ol class="check">
<li><b>Banner.</b> No new failures, and counts as expected? A new failure needs an issue.</li>
<li><b>Each test case, failures first:</b> read the purpose and the <i>conditions</i> table (initial state, loads, commands) before the plots; then check each plot against its dashed limits inside the shaded window.</li>
<li><b>Changes</b> since the previous campaign: each one needs a reason you know.</li>
<li><b>Sign off</b> in <code>record.md</code>: <code>reviewed: &lt;name&gt;, &lt;date&gt;, OK | questions</code>.</li></ol>''')

    for c in cases:
        f = c['folder']
        failed_here = any(r['verdict'] == 'FAIL' for r in c['rows'])
        p.append(f'<h2 id="{e(f.name)}">{"✗ " if failed_here else ""}{e(c["title"])} <span class="sub">({e(f.name)}/)</span></h2>')
        if c['purpose']:
            p.append(f'<p class="purpose">{e(c["purpose"])}</p>')
        p.append('<h3>1 · Test conditions and objectives</h3><div class="wrap"><table class="kv">')
        for k, v in c['conditions']:
            p.append(f'<tr><td>{e(k)}</td><td>{v}</td></tr>')
        p.append('</table></div>')
        if c['rows']:
            p.append('<table><tr><th>Objective</th><th>What is checked</th><th>Limit</th></tr>')
            for r in sorted(c['rows'], key=lambda r: r['id']):
                p.append(f'<tr><td>{e(r["id"])}</td><td>{e(r["desc"])}</td><td>{e(r["limit"])}</td></tr>')
            p.append('</table>')
        if c.get('kind') == 'target':
            p.append('<h3>2 · Target snapshot</h3><p class="sub">On-target test (ADR 0005): the board and firmware it ran on, '
                     'in place of a model.</p><div class="wrap"><table class="kv">')
            for k, v in c['target']:
                p.append(f'<tr><td>{e(k)}</td><td>{e(v)}</td></tr>')
            p.append('</table></div>')
        else:
            p.append('<h3>2 · Model snapshot</h3>')
            p.append(snapshot_html(f, c['snapshot_error'], c['changes']))
        p.append('<h3>3 · Results</h3><table><tr><th>Criterion</th><th>Verdict</th><th>Measured</th></tr>')
        for r in sorted(c['rows'], key=sort_key):
            p.append(f'<tr><td>{e(r["id"])}</td><td>{verdict_badge(r)}</td><td>{e(r["measured"])}</td></tr>')
        p.append('</table>')
        if c.get('kind') == 'target':
            # On-target timing (L0-TIM): reporting rules in tests/l0/timing/README.md "Reporting".
            rep_t = json.loads(c['report'].read_text()) if c['report'].exists() else None
            if not (rep_t and rep_t.get('rows')):
                p.append(f'<p class="sub">No report in {e(f.name)}/.</p>')
                continue
            xmax = timing_xmax(rep_t)
            groups_hint = ('One hue per group (blue boost, orange inverter logic, green calculateDuty, yellow observer, '
                           'pink scope), its sections in shades of it in code order, darkest first')
            if c.get('figures') == 'tim02':
                p.append(tim02_tables(rep_t))
                p.append('<h3>4 · Figures</h3>')
                rows_t = {r['name']: r for r in rep_t['rows']}
                groups = series_groups(rows_t, rep_t['buckets'][0]['name'], 'tim02')
                bars = paired_bars(rep_t)
                p.append('<figure><div><b>Overview: L0-TIM-01 against L0-TIM-02</b></div>' + timing_overview(bars, groups, xmax)
                         + '<figcaption>Per bucket, the L0-TIM-01 bar (controlStep sections and scope, main thread, '
                         'interrupts locked, scope on every tick) above the L0-TIM-02 bar (the whole tick in the critical '
                         'task: sensor reads in dark green, the same sections, PWM and relay writes in violet, scope at its '
                         f'production rate). {groups_hint}. Same scale and colours as the L0-TIM-01 figures. Black mark: '
                         'the worst case (TIM-01: worst controlStep + worst scope; TIM-02: worst whole tick). Dashed: the '
                         '80 µs budget and the 100 µs period.</figcaption></figure>')
                p.append('<figure><div><b>Detailed view: L0-TIM-01 against L0-TIM-02</b></div>' + timing_detail(bars, groups, xmax)
                         + '<figcaption>The same bars at 40 px per µs: every segment carries its name and mean in µs, so '
                         'a section reads directly against its L0-TIM-01 value just above it; scroll right, the bucket '
                         'names stay pinned. Hover a segment for its worst case.</figcaption></figure>')
                p.append('<figure><div><b>Event ticks</b></div>' + tim02_event_figure(rep_t) + '<figcaption>The worst '
                         'whole tick on which each event happened, per bucket (several events can share one tick). Red '
                         'with "over": above the 80 µs budget. The event table above is the same data.</figcaption></figure>')
                continue
            p.append(timing_table(rep_t, TIM01_COLS))
            p.append('<h3>4 · Figures</h3>')
            rows_t = {r['name']: r for r in rep_t['rows']}
            groups = series_groups(rows_t, rep_t['buckets'][0]['name'], 'tim01')
            bars = bar_rows(rep_t, 'tim01')
            p.append('<figure><div><b>Overview</b></div>' + timing_overview(bars, groups, xmax) + '<figcaption>Each bar: '
                     f'the mean cost of one tick in that bucket, one segment per code section. {groups_hint}. Values '
                     'where they fit; the detailed view labels every segment. Black mark: the worst controlStep plus the '
                     'worst scope sampling of the bucket. Dashed: the 80 µs budget and the 100 µs period. The time axis '
                     'is the one of the L0-TIM-02 figures below.</figcaption></figure>')
            p.append('<figure><div><b>Detailed view</b></div>' + timing_detail(bars, groups, xmax) + '<figcaption>The '
                     'same data at 40 px per µs: every segment carries its name and mean in µs; scroll right, the bucket '
                     'names stay pinned. Hover a segment for its worst case. Interrupts locked, no sensors or PWM: L0-TIM-02 '
                     'measures the whole tick. The table above is the same data.</figcaption></figure>')
            continue
        p.append('<h3>4 · Figures</h3>')
        figs = figures_for[f.name]
        if not figs:
            p.append('<p class="sub">No review plot is declared for this test case (tests/l1/review_plots.json).</p>')
        for r, fig_html, notes in figs:
            p.append(f'<figure><div><b>{e(r["id"])}</b> {verdict_badge(r)} {e(r["desc"])}</div>')
            if fig_html:
                p.append(fig_html)
                p.append('<figcaption>' + ' '.join(e(n) for n in notes) + '</figcaption>')
            else:
                p.append(f'<figcaption>No plot: {e(str(notes))}</figcaption>')
            p.append('</figure>')

    rest = [r for r in rows if r['id'] not in claimed]
    if rest:
        p.append('<h2 id="other">Criteria without a test-case folder</h2><p class="sub">L0 tests, '
                 'L1 checks that run no plant scenario, fixture checks and cases still BLOCKED.</p>')
        p.append('<div class="wrap"><table><tr><th>Criterion</th><th>Verdict</th><th>Description</th><th>Limit</th><th>Measured</th></tr>')
        for r in sorted(rest, key=sort_key):
            p.append(f'<tr><td>{e(r["id"])}</td><td>{verdict_badge(r)}</td><td>{e(r["desc"])}</td><td>{e(r["limit"])}</td><td>{e(r["measured"])}</td></tr>')
        p.append('</table></div>')

    p.append('<h2>Changed since the previous campaign</h2>')
    if not prev:
        p.append('<p class="sub">No earlier campaign of the same scope in tests/out/.</p>')
    elif not changes:
        p.append('<p class="sub">Nothing changed: same verdicts, measured values within 1 %.</p>')
    else:
        p.append('<div class="wrap"><table><tr><th>Criterion</th><th>Change</th><th>Before</th><th>Now</th></tr>')
        for r, b, why in changes:
            p.append(f'<tr><td>{e(r["id"])}</td><td>{e(why)}</td><td>{e(b["measured"]) if b else "—"}</td><td>{e(r["measured"])}</td></tr>')
        p.append('</table></div>')
    p.append(assets() + '</main></body></html>')
    (out / 'report.html').write_text('\n'.join(p), encoding='utf-8')
    print(f'Review: {(out / "report.html").relative_to(REPO) if (out / "report.html").is_relative_to(REPO) else out / "report.html"}')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
