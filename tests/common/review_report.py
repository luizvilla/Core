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
##      Bench test cases (a folder with bench_manifest.json, L2, tests/l2/bench.py) have the same four
##      parts: catalogue criteria and conditions; bench snapshot (firmware, operator, scope setup per
##      capture point) in place of the model snapshot; results; scope screenshots and waveforms, and one
##      interactive figure per firmware scope dump (key r), decoded from the raw dump, time 0 at the trigger.
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
    for man in sorted(out.glob('*/allocation_manifest.json')):
        folder = man.parent
        meta = json.loads(man.read_text())
        prefixes = tuple(meta.get('criteria') or [])
        cases.append({'folder': folder, 'kind': 'allocation', 'title': meta.get('title', folder.name),
                      'purpose': meta.get('purpose'), 'rows': [r for r in rows if r['id'].startswith(prefixes)],
                      'conditions': [(k, html.escape(v)) for k, v in meta.get('conditions', [])],
                      'shields': meta.get('shields', []), 'order': 'zy_' + folder.name})
    for man in sorted(out.glob('*/bench_manifest.json')):
        folder = man.parent
        meta = json.loads(man.read_text())
        cat_t = catalogue_entry(meta['id'])
        session = json.loads((out / 'bench_session.json').read_text()) if (out / 'bench_session.json').exists() else {}
        mine = [r for r in rows if r['id'].startswith(meta['id'] + '.')]
        e = html.escape
        cond = [('Criteria (catalogue)', e(cat_t.get('criterion', ''))),
                ('Conditions (catalogue)', e(cat_t.get('conditions', ''))),
                ('Procedure', e(cat_t.get('source', '')))]
        if meta.get('started'):
            cond.append(('Step window', e(f'{meta["started"]} to {meta.get("ended") or "(not finished)"}: telemetry criteria '
                                         'are judged on the console lines in this window')))
        for name, d, cap, pt, n_caps in bench_points(folder, meta):
            st = pt.get('settings') or {}
            st_txt = ', '.join(([f'Kp {st["kp"]:g} V/A'] if 'kp' in st else []) + ([f'Ti {st["ti_ms"]:g} ms'] if 'ti_ms' in st else [])
                               + (['feedforward ' + ('SOGI' if st['ff'] else 'measured')] if 'ff' in st else [])
                               + ([f'dead time {st["dt_ns"]} ns'] if 'dt_ns' in st else [])
                               + (['feedforward filter ' + ('on' if st['ffn'] else 'off')] if 'ffn' in st else [])
                               + (['DC loop ' + ('on' if st['dcl'] else 'off')] if 'dcl' in st else []))
            cond.append((f'Capture point {name}', e(f'{pt["timestamp"]}: {pt["stats_note"]}' + (f'; firmware {st_txt}' if st_txt else '')
                                                   + (f' ({n_caps} captures, this one {"pinned" if pt.get("_pinned") else "the newest"})' if n_caps > 1 else ''))))
        chans = session.get('channels') or {}
        if chans:
            cond.append(('Scope channels', '<br>'.join(e(f'{ch}: {c.get("signal", "")} ({c.get("kind", "")}) {c.get("label", "")}')
                                                       for ch, c in sorted(chans.items()))))
        for n in meta.get('notes', []):
            cond.append((f'Operator note {n["time"]}' + (f' ({n["point"]})' if n.get('point') else ''), e(n['text'])))
        cases.append({'folder': folder, 'kind': 'bench', 'title': f'{meta["id"]} · {cat_t.get("title", "")}',
                      'purpose': None, 'rows': mine, 'conditions': cond, 'bench': meta, 'session': session,
                      'order': 'y_' + folder.name})
    # A criterion judged on several runs (the PV 25/50 ns pair) is listed under each of them.
    return cases


def catalogue_entry(test_id):
    """@brief The catalogue entry of a test, or {} (the page never fails on a catalogue problem)."""
    try:
        from catalogue import load
        return load()['by_id'].get(test_id, {})
    except Exception:  # noqa: BLE001
        return {}


def eng(x, unit=''):
    """@brief Engineering notation (k, m, µ…) with 4 significant digits; '—' for None."""
    if x is None:
        return '—'
    for f, pre in ((1e9, 'G'), (1e6, 'M'), (1e3, 'k'), (1, ''), (1e-3, 'm'), (1e-6, 'µ'), (1e-9, 'n')):
        if abs(x) >= f:
            return f'{x / f:.4g} {pre}{unit}'.strip()
    return f'{x:.4g} {unit}'.strip()


def bench_points(f, meta):
    """@brief (name, folder, capture dict, capture entry, number of captures) per point: the evidence capture
    (pinned, else the newest), as tests/l2/session.py judges it."""
    out = []
    for name, pt in meta['points'].items():
        caps = [pt] if 'dir' in pt else pt.get('captures', [])
        if not caps:
            continue
        i = pt.get('pinned')
        ev = dict(caps[i] if i is not None and i < len(caps) else caps[-1], _pinned=i is not None)
        d = f / ev['dir']
        cap = json.loads((d / 'capture.json').read_text()) if (d / 'capture.json').exists() else None
        out.append((name, d, cap, ev, len(caps)))
    return out


def bench_console(out, meta):
    """@brief Console lines of the step window (console.txt), keys marked; at most 400 lines."""
    log = out / 'console.txt'
    if not (log.exists() and meta.get('started')):
        return ''
    import datetime as dt
    t0 = dt.datetime.fromisoformat(meta['started'])
    t1 = dt.datetime.fromisoformat(meta['ended']) if meta.get('ended') else None
    lines = []
    for ln in log.read_text(encoding='utf-8', errors='replace').splitlines():
        parts = ln.split(' ', 2)
        try:
            t = dt.datetime.fromisoformat(parts[0])
        except (ValueError, IndexError):
            continue
        if t >= t0 and (t1 is None or t <= t1) and len(parts) == 3:
            lines.append(f'{t:%H:%M:%S} ' + (f'» key {parts[2]}' if parts[1] == '>' else parts[2]))
    if not lines:
        return ''
    cut = len(lines) > 400
    body = html.escape('\n'.join(lines[:200] + ['…'] + lines[-200:] if cut else lines))
    return (f'<details><summary>Console in the step window ({len(lines)} lines{", middle elided: console.txt has all" if cut else ""})'
            f'</summary><pre style="max-height:420px;overflow:auto;font-size:12px">{body}</pre></details>')


def bench_snapshot(f, meta, session):
    """@brief Bench snapshot part: firmware and operator, then the scope setup of each capture point."""
    e = html.escape
    p = ['<div class="wrap"><table class="kv">']
    for k, label in (('firmware', 'Firmware on the board'), ('repo_commit', 'Repository at the session start'),
                     ('operator', 'Operator'), ('started', 'Session started'), ('note', 'Session note')):
        if session.get(k):
            p.append(f'<tr><td>{label}</td><td>{e(str(session[k]))}</td></tr>')
    pts = bench_points(f, meta)
    idn = next((c['setup']['idn'] for _, _, c, _, _ in pts if c), None)
    if idn:
        p.append(f'<tr><td>Oscilloscope</td><td>{e(idn)}</td></tr>')
    p.append('</table></div>')
    if pts:
        p.append('<div class="wrap"><table><tr><th>Point</th><th>Captured</th><th>Acquisition</th><th>Timebase</th>'
                 '<th>Trigger</th><th>Channels</th></tr>')
        for name, _, cap, pt, _ in pts:
            if not cap:
                p.append(f'<tr><td>{e(name)}</td><td colspan="5">capture.json missing</td></tr>')
                continue
            s = cap['setup']
            h, t, a = s['horizontal'], s['trigger'], s['acquire']
            chans = '<br>'.join(e(f'{ch}{" " + c["label"] if c.get("label") else ""}: {eng(c["scale_div"], c.get("probe_units") or "V")}/div, '
                                  f'{c.get("coupling")}, {eng(c.get("bandwidth"), "Hz")}') for ch, c in s['channels'].items())
            p.append(f'<tr><td>{e(name)}</td><td>{e(pt["timestamp"])}</td><td>{e(str(a.get("mode")))}, {e(pt["stats_note"])}</td>'
                     f'<td>{eng(h.get("scale_s_div"), "s")}/div, {eng(h.get("sample_rate"), "S/s")}, {h.get("record_length") or 0:.0f} pts</td>'
                     f'<td>{e(str(t.get("type")))} {e(str(t.get("source")))} {e(str(t.get("slope")))} @ {eng(t.get("level"), "V")}, {e(str(t.get("mode")))}</td>'
                     f'<td>{chans}</td></tr>')
        p.append('</table></div>')
    return '\n'.join(p)


def bench_figures(f, meta):
    """@brief Figures part of a bench case: per point the scope screenshot, its measurement table (the table
    view) and, when stored, the waveforms (one panel per unit, channels in the fixed palette order)."""
    e = html.escape
    p = []
    pts = bench_points(f, meta)
    if not pts:
        p.append('<p class="sub">No oscilloscope capture.</p>')
    for name, d, cap, pt, n_caps in pts:
        rel = f'{f.name}/{pt["dir"]}'
        if (d / 'screen.png').exists():
            p.append(f'<figure><div><b>Point {e(name)}</b> <span class="sub">{e(pt["timestamp"])}</span></div>'
                     f'<a href="{e(rel)}/screen.png"><img alt="Oscilloscope screen, {e(meta["id"])} point {e(name)}" src="{e(rel)}/screen.png"></a>'
                     f'<figcaption>Oscilloscope screen of the evidence capture{" (pinned by the operator)" if pt.get("_pinned") else ""}'
                     f'{f", one of {n_caps} captures of this point (all in {e(f.name)}/captures/)" if n_caps > 1 else ""} '
                     f'(click for full size). Statistics: {e(pt["stats_note"])}. '
                     f'Raw data: <code>{e(rel)}/</code>.</figcaption></figure>')
        if cap and cap.get('measurements'):
            stat = pt.get('stats', 'all')
            p.append(f'<details><summary>Scope measurements at point {e(name)} '
                     f'({"over the statistics window" if stat == "all" else "last acquisition"})</summary><div class="wrap"><table>'
                     '<tr><th>Meas</th><th>Type</th><th>Source</th><th>Mean</th><th>Min</th><th>Max</th><th>σ</th><th>N</th></tr>')
            for m in cap['measurements']:
                u, r = m.get('unit') or '', m[stat]
                n = '' if r.get('popula') is None else f'{r["popula"]:.0f}'
                p.append(f'<tr><td>{e(m["id"])}</td><td>{e(str(m["type"]))}</td><td>{e(str(m["source1"]))}</td>'
                         f'<td>{eng(r.get("mean"), u)}</td><td>{eng(r.get("minimu"), u)}</td><td>{eng(r.get("maximu"), u)}</td>'
                         f'<td>{eng(r.get("stddev"), u)}</td><td>{n}</td></tr>')
            p.append('</table></div></details>')
        if (d / 'waveforms.npz').exists() and cap:  # stored for checking; verdicts come from the scope's measurements
            import numpy as np
            w = np.load(d / 'waveforms.npz')
            chans = cap['setup']['channels']
            fig = Figure('ms')
            by_unit = {}
            for ch in chans:
                if f'{ch}_v' in w.files:
                    by_unit.setdefault(chans[ch].get('probe_units') or 'V', []).append(ch)
            for unit, chs in by_unit.items():
                c = fig.chart(unit, 220)
                for ch in chs:
                    fig.line(c, ch + (f' {chans[ch]["label"]}' if chans[ch].get('label') else ''),
                             w[f'{ch}_t'] * 1e3, w[f'{ch}_v'], SERIES[(int(ch[2:]) - 1) % len(SERIES)])
            fid = re.sub(r'[^A-Za-z0-9_-]', '_', f'{f.name}-{name}-wf')
            p.append(f'<figure><div><b>Waveforms at point {e(name)}</b></div>' + fig.html(fid, f'{meta["id"]} waveforms, point {name}')
                     + '<figcaption>Displayed channels as acquired, one panel per unit; CH1 blue, CH2 orange, CH3 green, '
                     'CH4 yellow.' + (' Long records are reduced to each bucket\'s min and max, so peaks survive.' if fig.reduced else '')
                     + '</figcaption></figure>')
    p += bench_firmware_scope(f, meta)
    p.append(bench_console(f.parent, meta))
    return '\n'.join(p)


# Firmware scope panels (src/main.cpp channel names): one panel per quantity, so each axis stays readable.
FW_PANELS = [('V', ['VAC_V', 'V_d_V', 'Vd_avg_V']), ('V, quadrature', ['V_q_V', 'Vq_avg_V']), ('A', ['IAC_A', 'ILow1_A']),
             ('rad/s', ['omega_rad_s', 'omega_avg_rad_s']), ('state', ['relay_cmd', 'Sync', 'inverter_mode']),
             ('duty', ['bridge_duty']), ('V, DC bus', ['VDCBus_V'])]


def bench_firmware_scope(f, meta):
    """@brief One figure per firmware scope dump of the step (key r): the micro-controller's own record, decoded
    from the raw dump (oldest sample first), time 0 at the trigger. Rate and trigger sample: as recorded with the
    dump (the firmware prints them), else from the session console (console.fw_timebase)."""
    import datetime as dt
    import numpy as np
    sys.path.insert(0, str(REPO / 'tests' / 'l2'))
    from console import fw_timebase, parse_dump, read_log
    e = html.escape
    caps = meta.get('firmware_captures') or []
    if not caps:
        return []
    camp = f.parent
    rx = None
    out = ['<h4>Firmware scope (key r)</h4>']
    for n, fc in enumerate(caps):
        txt, csvp = camp / fc['txt'], camp / fc['csv'] if fc.get('csv') else None
        got = parse_dump(txt.read_text().splitlines()) if txt.exists() else None
        if not got and csvp and csvp.exists():
            rows = list(csv.reader(csvp.open()))
            got = (rows[0], [[float(x) for x in r] for r in rows[1:] if len(r) == len(rows[0])])
        link = f'<a href="{e(fc.get("csv") or fc["txt"])}">{e(Path(fc.get("csv") or fc["txt"]).name)}</a>'
        if not got:
            out.append(f'<p class="sub">Firmware scope {e(fc["time"][11:])}: dump not readable ({link}).</p>')
            continue
        names, rows = got
        if fc.get('rate_hz') and fc.get('trigger_index'):
            rate, trig, why = fc['rate_hz'], fc['trigger_index'], fc.get('rate_why', 'recorded with the dump')
        else:
            if rx is None:
                rx = read_log(camp / 'console.txt')[0]
            rate, trig, why = fw_timebase(rx, dt.datetime.fromisoformat(fc['time']).timestamp())
        data = np.asarray(rows, float)
        t = (np.arange(len(data)) - trig) / rate * 1e3
        fig = Figure('ms')
        used = set()
        panels = [(u, [x for x in chs if x in names]) for u, chs in FW_PANELS]
        panels += [(nm, [nm]) for nm in names if not any(nm in chs for _, chs in FW_PANELS)]
        for unit, chs in panels:
            if not chs:
                continue
            c = fig.chart(unit, 170 if unit != 'state' else 130)
            for i, nm in enumerate(chs):
                fig.line(c, nm, t, data[:, names.index(nm)], SERIES[i % len(SERIES)], 1.6)
                used.add(nm)
            c['events'] = [{'t': 0.0, 'label': 'trigger'}]
        fid = re.sub(r'[^A-Za-z0-9_-]', '_', f'{f.name}-fw-{n}')
        st = fc.get('settings') or {}
        st_txt = ', '.join(([f'Kp {st["kp"]:g} V/A'] if 'kp' in st else []) + ([f'Ti {st["ti_ms"]:g} ms'] if 'ti_ms' in st else [])
                           + (['feedforward ' + ('SOGI' if st['ff'] else 'measured')] if 'ff' in st else [])
                               + ([f'dead time {st["dt_ns"]} ns'] if 'dt_ns' in st else [])
                               + (['feedforward filter ' + ('on' if st['ffn'] else 'off')] if 'ffn' in st else [])
                               + (['DC loop ' + ('on' if st['dcl'] else 'off')] if 'dcl' in st else []))
        cap = (f'Firmware scope at {e(fc["time"][11:])}' + (f' (firmware {e(st_txt)})' if st_txt else '') + f': {len(data)} samples at {rate / 1000:g} kHz '
               f'({len(data) / rate * 1e3:.1f} ms), time 0 at the trigger (sample {trig}). Rate: {e(why)}. '
               f'Click a legend entry to hide or show that signal. Raw data: {link}.')
        body = (f'<figure>' + fig.html(fid, f'{meta["id"]} firmware scope {fc["time"][11:]}') + f'<figcaption>{cap}</figcaption></figure>')
        last = n == len(caps) - 1
        out.append(f'<details{" open" if last else ""}><summary><b>Firmware scope {e(fc["time"][11:])}</b>'
                   f'{" (newest)" if last and len(caps) > 1 else ""}</summary>{body}</details>')
    return out


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
# L0-TIM-03: work the task API does in the same interrupt before the task (low-chroma blue-grey, apart from the rest).
TASKAPI_GROUP = ('task API (same interrupt)', '#4a5a6a', [('3:safety', 'safety check'), ('3:dispatch', 'ADC data dispatch')])
TIMING_SHORT.update({'ap_boost_duty': 'b.duty', 'ap_boost_en': 'b.en', 'ap_burst': 'burst', 'ap_bridge_duty': 'br.duty',
                     'ap_bridge_en': 'br.en', 'ap_pwm_ack': 'ack', 'ap_relay': 'relay'})
TIMING_SHORT.update({'tick_sensors': 'sensors', 'tick_apply': 'writes', 'tick_scope': 'scope', '_rest': 'rest',
                     'tick_control': 'controlStep'})
# L0-TIM-03 ADC data path (owntech_timing_probe.h): the dispatch per ADC and the latest-value reads per layer. Derived
# segments (a measured total minus its probed parts) keep each bar equal to its measured total.
TASKAPI_ADC_SECS = [('3:safety', 'safety check'), ('3:disp_adc1', 'dispatch ADC1'), ('3:disp_adc2', 'dispatch ADC2'),
                    ('3:dispatch_rest', 'dispatch, other ADCs + loop')]
SENSOR_LAYER_SECS = [('3:rd_sensor', 'lookup + checks'), ('3:rd_buffer', 'buffer layer'), ('3:rd_convert', 'conversion'),
                     ('3:sensors_rest', 'around the reads')]
DERIVED = {'3:dispatch_rest': ('3:dispatch', ('3:disp_adc1', '3:disp_adc2')),
           '3:sensors_rest': ('3:tick_sensors', ('3:rd_sensor', '3:rd_buffer', '3:rd_convert'))}
COUNT_ROWS = ('3:n_adc1', '3:n_adc2')  # sample counts, not cycles
TIMING_SHORT.update({'disp_adc1': 'ADC1', 'disp_adc2': 'ADC2', 'dispatch_rest': 'rest', 'rd_sensor': 'lookup',
                     'rd_buffer': 'buffer', 'rd_convert': 'conv', 'sensors_rest': 'rest'})


def series_groups(rows, bucket, kind):
    """@brief Groups of one test, with the row keys it reports. L0-TIM-01: the controlStep sections and the scope.
    L0-TIM-02: sensor reads, the same sections measured in the critical tick ('2:' rows), PWM and relay writes,
    scope, in the order the tick runs them. Shared groups keep their hue and shades in both tests."""
    base = timing_groups(rows, bucket)
    if kind == 'tim01':
        return base + [REST_GROUP]
    p = '3:' if kind == 'tim03' else ''   # tick-part keys: 'tick_*' (TIM-02) or '3:tick_*' (TIM-03)
    sp = '3:' if kind == 'tim03' else '2:'
    adc = kind == 'tim03' and f'{bucket}/3:disp_adc1' in rows  # a run with the ADC path probes
    out = ([(TASKAPI_GROUP[0], TASKAPI_GROUP[1], TASKAPI_ADC_SECS)] if adc else [TASKAPI_GROUP]) if kind == 'tim03' else []
    out.append((SENSOR_GROUP[0], SENSOR_GROUP[1], SENSOR_LAYER_SECS if adc else [(p + 'tick_sensors', 'sensor reads')]))
    if f'{bucket}/{sp}boost_cold' in rows:
        for gname, hue, secs in base:
            if gname != 'scope':
                out.append((gname, hue, [(sp + k, t) for k, t in secs]))
    else:  # a run recorded before TIM-02 kept the sections: controlStep as one block, never dropped
        out.append(('controlStep', '#1baf7a', [(p + 'tick_control', 'controlStep (not split in this run)')]))
    if kind == 'tim03' and f'{bucket}/3:ap_boost_duty' in rows:  # the writes split by call (apply probes)
        writes = [('3:ap_boost_duty', 'boost duty'), ('3:ap_boost_en', 'boost enable'), ('3:ap_burst', 'burst'),
                  ('3:ap_bridge_duty', 'bridge duty'), ('3:ap_bridge_en', 'bridge enables'),
                  ('3:ap_pwm_ack', 'PWM ack'), ('3:ap_relay', 'relay')]
    else:
        writes = [(p + 'tick_apply', 'PWM + relay writes')]
    out += [(WRITE_GROUP[0], WRITE_GROUP[1], writes),
            ('scope', '#e87ba4', [(p + 'tick_scope', 'snapshot + acquire')]), REST_GROUP]
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
        elif kind == 'tim03':  # the interrupt's work per period without the harness: task API + task tick
            api = mean_of('3:safety') + mean_of('3:dispatch')
            total = api + mean_of('3:tick')
            worst = api + u(rows[f'{b}/3:tick']['max_cycles'])
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
                if key in DERIVED:
                    whole, parts = DERIVED[key]
                    v = mean_of(whole) - sum(mean_of(x) for x in parts)
                    segs.append((gname, hue, k, len(secs), '_rest', text, max(v, 0.0), max(v, 0.0)))
                    continue
                r = rows.get(f'{b}/{key}')
                if r and r['calls']:
                    segs.append((gname, hue, k, len(secs), key, text, u(r['sum_cycles'] / r['calls']), u(r['max_cycles'])))
        out.append({'bucket': b, 'tag': tag, 'segs': segs, 'worst': worst, 'mean': total})
    return out


def timing_xmax(rep, period_us=100.0):
    """@brief Common time axis of every timing figure of a report, so TIM-01 and TIM-02 read on the same scale."""
    worst = max([r['worst'] for r in bar_rows(rep, 'tim01')] +
                ([r['worst'] for r in bar_rows(rep, 'tim02')] if any(n['name'].endswith('/tick') for n in rep['rows']) else []) +
                ([r['worst'] for r in bar_rows(rep, 'tim03')] if any(n['name'].endswith('/3:tick') and n['calls'] for n in rep['rows']) else []))
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


def paired_bars(rep, first='tim01', second='tim02'):
    """@brief Per bucket, the bar of the earlier test then the bar of the later one."""
    a, b = bar_rows(rep, first, first.upper().replace('TIM', 'TIM-')), bar_rows(rep, second, second.upper().replace('TIM', 'TIM-'))
    return [x for pair in zip(a, b) for x in pair]


def tim03_gap_figure(rep, limit_us=150.0, period_us=100.0):
    """@brief Inline SVG: per bucket, the mean and worst gap between two task starts, against the period and the
    150 µs limit, with the lost periods written at the end."""
    e = html.escape
    hz = rep['core_hz']
    rows = {r['name']: r for r in rep['rows']}
    u = lambda c: c * 1e6 / hz
    buckets = rep['buckets']
    W, left, right, top, bar, gap = 960, 130, 220, 30, 18, 10
    worst_all = max(u(rows[f"{b['name']}/3:gap"]['max_cycles']) for b in buckets)
    xmax = max(limit_us + 20, 10 * (int(worst_all / 10) + 2))
    H = top + len(buckets) * (bar + gap) + 30
    x = lambda v: left + (W - left - right) * min(v, xmax) / xmax
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" role="img" aria-label="Gap between task starts" '
           f'style="font:12px system-ui,sans-serif;background:{SURFACE}">']
    for v in range(0, int(xmax) + 1, 10):
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{top - 6}" y2="{H - 26}" stroke="{GRID}"/>'
                   + (f'<text x="{x(v):.1f}" y="{H - 10}" fill="{MUTED}" text-anchor="middle">{v} µs</text>' if v % 20 == 0 else ''))
    for v, label in ((period_us, f'period {period_us:g} µs'), (limit_us, f'limit {limit_us:g} µs')):
        svg.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{top - 10}" y2="{H - 26}" stroke="{INK2}" stroke-dasharray="6 4"/>'
                   f'<text x="{x(v) - 4:.1f}" y="{top - 14}" fill="{INK2}" text-anchor="end">{e(label)}</text>')
    for i, bk in enumerate(buckets):
        b = bk['name']
        y = top + i * (bar + gap)
        g = rows[f'{b}/3:gap']
        mean, worst = u(g['sum_cycles'] / max(g['calls'], 1)), u(g['max_cycles'])
        lost = bk.get('tim03_lost', 0)
        col = CRITICAL if worst > limit_us or lost else SERIES[0]
        svg.append(f'<text x="{left - 8}" y="{y + bar / 2 + 4}" fill="{INK}" text-anchor="end">{e(b)}</text>'
                   f'<rect x="{left}" y="{y}" width="{max(x(worst) - left, 1):.1f}" height="{bar}" rx="2" fill="{col}" opacity="0.35">'
                   f'<title>{e(b)}: worst gap {worst:.1f} µs</title></rect>'
                   f'<rect x="{left}" y="{y}" width="{max(x(mean) - left, 1):.1f}" height="{bar}" rx="2" fill="{col}">'
                   f'<title>{e(b)}: mean gap {mean:.1f} µs</title></rect>'
                   f'<text x="{x(max(worst, limit_us)) + 6:.1f}" y="{y + bar / 2 + 4}" fill="{INK}">mean {mean:.1f} · worst {worst:.1f} µs · '
                   f'{"<tspan font-weight=\"600\">" + str(lost) + " lost</tspan>" if lost else "0 lost"}</text>')
    svg.append('</svg>')
    return ''.join(svg)


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
            if r and r['calls'] and key in COUNT_ROWS:  # a count: shown as is
                cells.append(f"<td>{r['max_cycles']} <span class=\"sub\">({r['sum_cycles'] / r['calls']:.1f})</span></td>")
                continue
            cells.append('<td>—</td>' if not r or not r['calls'] else
                         f"<td>{u(r['max_cycles']):.1f} <span class=\"sub\">({u(r['sum_cycles'] / r['calls']):.1f})</span></td>")
        p.append(f'<tr><td>{e(b)}</td>' + ''.join(cells) + '</tr>')
    p.append('</table></div><p class="sub">Worst case, mean in brackets, in µs'
             + ('; the sample columns are counts of ADC samples dispatched per period' if any(k in COUNT_ROWS for k, _ in cols) else '')
             + '. Inverter states reached: '
             + '; '.join(f"{e(b['name'])} {e(', '.join(VSI_NAMES[m] for m in b.get(states_key, [])))}" for b in rep['buckets']) + '.</p>')
    return '\n'.join(p)


TIM01_COLS = [('boost_cold', 'boost_cold'), ('boost_hot', 'boost_hot'), ('inv_stage', 'inv_stage'), ('inv_duty', 'inv_duty'),
              ('duty_pll', 'duty_pll'), ('duty_sogi', 'duty_sogi'), ('duty_park', 'duty_park'), ('duty_syncwin', 'duty_syncwin'),
              ('duty_mode_ff', 'duty_mode_ff'), ('duty_current_pi', 'duty_current_pi'), ('duty_output', 'duty_output'),
              ('observer', 'observer'), ('outputs', 'outputs'), ('controlStep', 'controlStep'), ('scope', 'scope')]
TIM03_COLS = ([('3:gap', 'gap'), ('3:safety', 'safety'), ('3:dispatch', 'data dispatch'), ('3:disp_adc1', 'd: ADC1'),
               ('3:n_adc1', 'd: ADC1 samples'), ('3:disp_adc2', 'd: ADC2'), ('3:n_adc2', 'd: ADC2 samples'), ('3:tick', 'task tick'),
               ('3:tick_sensors', 'sensors'), ('3:rd_sensor', 's: lookup'), ('3:rd_buffer', 's: buffer'),
               ('3:rd_convert', 's: conversion')] + [('3:' + k, h) for k, h in TIM01_COLS if k not in ('controlStep', 'scope')]
              + [('3:tick_control', 'controlStep'), ('3:tick_apply', 'writes'), ('3:ap_boost_duty', 'w: boost duty'),
                 ('3:ap_boost_en', 'w: boost enable'), ('3:ap_burst', 'w: burst'), ('3:ap_bridge_duty', 'w: bridge duty'),
                 ('3:ap_bridge_en', 'w: bridge enables'), ('3:ap_pwm_ack', 'w: PWM ack'), ('3:ap_relay', 'w: relay'),
                 ('3:tick_scope', 'scope'), ('3:harness', 'harness (test)')])
TIM02_COLS = ([('tick', 'whole tick'), ('tick_sensors', 'sensors')] + [('2:' + k, h) for k, h in TIM01_COLS if k not in ('controlStep', 'scope')]
              + [('tick_control', 'controlStep'), ('tick_apply', 'writes'), ('tick_scope', 'scope')])


def allocation_figure(folder, shields):
    """@brief L0-ALC: one row per shield, one cell per HRTIM fault input, coloured by the path holding it.

    Blue: comparator, orange: ADC watchdog, grey: free. The cell names the limit and bound. The table below it
    gives the same data per limit.
    """
    e = html.escape
    W, label_w, cell_w, row_h, top = 960, 170, 130, 30, 34
    rows = [s for s in shields if s.get('builds') and (folder / f"{s['shield']}.json").exists()]
    H = top + row_h * len(rows) + 40
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" role="img" aria-label="HRTIM fault inputs per shield" '
           f'font-family="system-ui,sans-serif" font-size="11">']
    for k in range(6):
        svg.append(f'<text x="{label_w + k * cell_w + cell_w / 2}" y="{top - 10}" fill="{INK2}" text-anchor="middle">FLT{k + 1}</text>')
    for i, sh in enumerate(rows):
        man = json.loads((folder / f"{sh['shield']}.json").read_text())
        flt = man['resources']['flt']
        y = top + i * row_h
        svg.append(f'<text x="{label_w - 8}" y="{y + 19}" fill="{INK}" text-anchor="end">{e(sh["shield"])}</text>')
        for k in range(6):
            v = flt.get(f'FLT{k + 1}')
            fill = GRID if v is None else (SERIES[0] if '(COMP' in v else SERIES[1])
            txt = 'free' if v is None else v.split(' (')[0]
            x = label_w + k * cell_w
            svg.append(f'<rect x="{x + 2}" y="{y + 3}" width="{cell_w - 4}" height="{row_h - 6}" rx="3" fill="{fill}">'
                       f'<title>{e(sh["shield"])} FLT{k + 1}: {e(v or "free")}</title></rect>'
                       f'<text x="{x + cell_w / 2}" y="{y + 19}" text-anchor="middle" fill="{text_on(fill)}">{e(txt[:20])}</text>')
    ly = H - 14
    for j, (lab, col) in enumerate([('comparator', SERIES[0]), ('ADC watchdog', SERIES[1]), ('free', GRID)]):
        svg.append(f'<rect x="{label_w + j * 140}" y="{ly - 10}" width="12" height="12" fill="{col}" rx="2"/>'
                   f'<text x="{label_w + j * 140 + 18}" y="{ly}" fill="{INK2}">{lab}</text>')
    svg.append('</svg>')
    return ''.join(svg)


def allocation_tables(folder, shields):
    """@brief L0-ALC: the protection table of each shield (the same file as docs/safety/boards/)."""
    e = html.escape
    out = []
    for sh in shields:
        f = folder / f"{sh['shield']}.json"
        if not f.exists():
            out.append(f'<p class="sub">{e(sh["shield"])}: did not build, see {e(sh["shield"])}.log.</p>')
            continue
        man = json.loads(f.read_text())
        out.append(f'<details><summary>{e(man["shield"])} ({e(sh["shield"])})</summary><table>'
                   '<tr><th>Limit</th><th>Low</th><th>High</th><th>Low bound</th><th>High bound</th><th>Exception</th></tr>')
        for l in man['limits']:
            def pt(p):
                if not p:
                    return 'none'
                if p['kind'] == 'comparator':
                    return f"COMP{p['comp']} → FLT{p['flt']}"
                if p['kind'] == 'adc_watchdog':
                    return f"{p['adc'].upper()} AWD{p['awd']} → FLT{p['flt']}"
                return 'software'
            out.append(f'<tr><td>{e(l["sensor"])}</td><td>{l["low"]:g}</td><td>{l["high"]:g}</td>'
                       f'<td>{e(pt(l["paths"]["low"]))}</td><td>{e(pt(l["paths"]["high"]))}</td>'
                       f'<td>{e(l.get("exception") or "")}</td></tr>')
        out.append('</table></details>')
    return ''.join(out)


def supervision_cases_table(rep):
    """@brief L0-SUP: one row per case as the harness ran it (acknowledged, resets, time, fault latched at boot)."""
    e = html.escape
    names = {0: '-', 1: 'fatal error', 2: 'watchdog'}
    out = ['<table><tr><th>Case</th><th>Run</th><th>Resets</th><th>Time (s)</th><th>Latched after</th></tr>']
    for c in rep.get('cases', []):
        out.append(f'<tr><td>{e(c["case"])}</td><td>{e(c["status"])}</td><td>{c["boots"]}</td>'
                   f'<td>{c["seconds"]}</td><td>{e(names.get(c.get("latched_at_boot", 0), "?"))}</td></tr>')
    out.append('</table>')
    return ''.join(out)


def supervision_figure(rows):
    """@brief L0-SUP: each declared-time criterion (limit "<= x us") as its measured share of the limit.

    One bar per criterion on a common 0-120 % axis (the limits span 10 us to 1 ms, so absolute scales would hide
    the small ones); the dashed line is the declared limit. Values and limits are in the results table above.
    """
    e = html.escape
    items = []
    for r in sorted(rows, key=sort_key):
        lim = r.get('limit', '')
        if not lim.strip().startswith('<='):
            continue
        lv, mv = first_number(lim), first_number(r.get('measured', ''))
        if lv and mv is not None:
            items.append((r['id'], r.get('desc', ''), mv, lv, r['verdict']))
    if not items:
        return '<p class="sub">No declared-time criterion in this run.</p>'
    W, label_w, row_h, top = 960, 300, 34, 24
    plot_w = W - label_w - 120
    H = top + row_h * len(items) + 30
    x = lambda f: label_w + plot_w * min(f, 1.2) / 1.2
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" role="img" aria-label="Measured share of the declared limit" '
           f'font-family="system-ui,sans-serif" font-size="12">']
    for t in (0, 0.25, 0.5, 0.75, 1.0):
        svg.append(f'<line x1="{x(t):.1f}" y1="{top - 6}" x2="{x(t):.1f}" y2="{H - 24}" stroke="{GRID}"/>'
                   f'<text x="{x(t):.1f}" y="{H - 8}" fill="{MUTED}" text-anchor="middle">{int(t * 100)} %</text>')
    for i, (cid, desc, mv, lv, verdict) in enumerate(items):
        y = top + i * row_h
        frac = mv / lv
        fill = SERIES[0] if verdict == 'PASS' else CRITICAL
        short = desc.split(':')[0][:44]
        svg.append(f'<text x="{label_w - 8}" y="{y + 17}" fill="{INK}" text-anchor="end">{e(cid)} · {e(short)}</text>')
        svg.append(f'<rect x="{label_w}" y="{y + 6}" width="{max(x(frac) - label_w, 1.5):.1f}" height="16" fill="{fill}" rx="2">'
                   f'<title>{e(cid)}: {mv:g} of {lv:g} ({frac * 100:.1f} %)</title></rect>')
        svg.append(f'<text x="{x(frac) + 6:.1f}" y="{y + 18}" fill="{INK2}">{mv:g} / {lv:g} ({frac * 100:.1f} %)</text>')
    svg.append(f'<line x1="{x(1):.1f}" y1="{top - 10}" x2="{x(1):.1f}" y2="{H - 24}" stroke="{CRITICAL}" stroke-dasharray="5 4"/>'
               f'<text x="{x(1) + 4:.1f}" y="{top - 12}" fill="{CRITICAL}">declared limit</text></svg>')
    return ''.join(svg)


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
        elif c.get('kind') == 'allocation':
            p.append('<h3>2 · Build snapshot</h3><p class="sub">Devicetree and allocation only (no firmware, no board): '
                     'the shields checked and whether each built.</p><div class="wrap"><table class="kv">')
            for sh in c['shields']:
                p.append(f'<tr><td>{e(sh["shield"])}</td><td>{"builds" if sh.get("builds") else "does not build"}'
                         f'{"" if sh.get("docs_match", True) else ", committed table differs"}</td></tr>')
            p.append('</table></div>')
        elif c.get('kind') == 'bench':
            p.append('<h3>2 · Bench snapshot</h3><p class="sub">Bench test (level L2): the firmware and the oscilloscope '
                     'setup of each capture, in place of a model.</p>')
            p.append(bench_snapshot(f, c['bench'], c['session']))
        else:
            p.append('<h3>2 · Model snapshot</h3>')
            p.append(snapshot_html(f, c['snapshot_error'], c['changes']))
        p.append('<h3>3 · Results</h3><table><tr><th>Criterion</th><th>Verdict</th><th>Measured</th></tr>')
        for r in sorted(c['rows'], key=sort_key):
            p.append(f'<tr><td>{e(r["id"])}</td><td>{verdict_badge(r)}</td><td>{e(r["measured"])}</td></tr>')
        p.append('</table>')
        if c.get('kind') == 'allocation':
            p.append('<h3>4 · Figures</h3>')
            p.append('<figure><div><b>HRTIM fault inputs per shield</b></div>' + allocation_figure(f, c['shields'])
                     + '<figcaption>Each row a shield, each cell one of the six HRTIM fault inputs, coloured by the path '
                     'that holds it (comparator or ADC watchdog) and labelled with the limit; grey: free. A row with no '
                     'grey has no channel left: any further limit on that board needs an exception or a routing change. '
                     'The per-limit tables follow.</figcaption></figure>')
            p.append(allocation_tables(f, c['shields']))
            continue
        if c.get('kind') == 'bench':
            p.append('<h3>4 · Figures</h3>')
            p.append(bench_figures(f, c['bench']))
            continue
        if c.get('kind') == 'target':
            # On-target timing (L0-TIM): reporting rules in tests/l0/timing/README.md "Reporting".
            rep_t = json.loads(c['report'].read_text()) if c['report'].exists() else None
            if c.get('figures') == 'supervision':
                # L0-SUP (ADR 0009): cases as run, then each declared time against its limit.
                if rep_t:
                    p.append(supervision_cases_table(rep_t))
                p.append('<h3>4 · Figures</h3>')
                p.append('<figure><div><b>Measured against the declared fault reaction time</b></div>'
                         + supervision_figure(c['rows'])
                         + '<figcaption>One bar per criterion with a declared limit ("<= x us"): the measured value as a '
                         'share of that limit, on one axis because the limits span 10 us to 1 ms. Dashed: the declared '
                         'limit. Blue: pass, red: fail. Hover a bar for its values; the results table above lists them '
                         'too.</figcaption></figure>')
                continue
            if not (rep_t and rep_t.get('rows')):
                p.append(f'<p class="sub">No report in {e(f.name)}/.</p>')
                continue
            xmax = timing_xmax(rep_t)
            groups_hint = ('One hue per group (blue boost, orange inverter logic, green calculateDuty, yellow observer, '
                           'pink scope), its sections in shades of it in code order, darkest first')
            if c.get('figures') == 'tim03':
                p.append(timing_table(rep_t, TIM03_COLS, 'tim03_vsi_seen'))
                p.append('<p class="sub">Lost periods per bucket: ' + ', '.join(f"{e(b['name'])} {b.get('tim03_lost', 0)}" for b in rep_t['buckets'])
                         + '. The harness row is the test\'s own cost in the interrupt; the figures leave it out.</p>')
                p.append('<h3>4 · Figures</h3>')
                rows_t = {r['name']: r for r in rep_t['rows']}
                groups = series_groups(rows_t, rep_t['buckets'][0]['name'], 'tim03')
                bars = paired_bars(rep_t, 'tim02', 'tim03')
                p.append('<figure><div><b>Overview: L0-TIM-02 against L0-TIM-03</b></div>' + timing_overview(bars, groups, xmax)
                         + '<figcaption>Per bucket, the L0-TIM-02 bar (the tick body alone in the critical task) above the '
                         'L0-TIM-03 bar (the interrupt\'s work per period in the running firmware: the task API\'s safety check '
                         'and ADC data dispatch in blue-grey, then the task tick with its own load bookkeeping in grey at the '
                         f'end). {groups_hint}; sensor reads dark green, writes violet. The test harness\'s own time is left out '
                         '(table above). Black mark: worst (TIM-02: whole tick; TIM-03: mean task API + worst task tick). '
                         'Dashed: the 80 µs budget and the 100 µs period.</figcaption></figure>')
                p.append('<figure><div><b>Detailed view: L0-TIM-02 against L0-TIM-03</b></div>' + timing_detail(bars, groups, xmax)
                         + '<figcaption>The same bars at 40 px per µs, every segment labelled; scroll right, bucket names '
                         'pinned.</figcaption></figure>')
                p.append('<figure><div><b>Gap between two task starts</b></div>' + tim03_gap_figure(rep_t) + '<figcaption>Solid: '
                         'mean gap, light: worst gap, per bucket; the lost periods (accumulated lateness of one period each) '
                         'at the end. A mean above the period means the task cannot keep up: periods are lost even when no '
                         'single gap reaches the limit. Red: over the limit or periods lost.</figcaption></figure>')
                continue
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
