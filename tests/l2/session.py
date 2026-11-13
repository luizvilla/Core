## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
## @file session.py
## @brief L2 bench session model: campaign files, scope measurement plan, judging, bench sheet text.
## @par Campaign tests/out/<date>_<sha>_<time>_bench/ (tests/out/bench-current points at the open one)
##   bench_session.json   operator, firmware, note, channel map (CHn → signal), measurements the app created
##   console.txt          serial console (tests/l2/console.py), firmware_scope/ its dumps (key r)
##   l2_<obj><nn>/        one folder per test: bench_manifest.json, captures/<time>_<point>/ (scope.capture())
##   l2_<obj><nn>.log     result lines of the test, rewritten on every change; record.md, report.html
## @note A criterion is judged on the pinned capture of its point, else the newest. Telemetry is judged
##   over the step window: from Start step (or the first key sent while the step is open) to Finish step.
"""Bench session: everything the app and bench.py read and write, and how a criterion is judged."""
from __future__ import annotations

import datetime as dt
import html
import json
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
OUT = REPO / 'tests' / 'out'
CURRENT = OUT / 'bench-current'
CRITERIA = HERE / 'criteria.json'
sys.path.insert(0, str(REPO / 'tests' / 'common'))
sys.path.insert(0, str(HERE))
from catalogue import load as load_catalogue  # noqa: E402
from console import parse_telemetry, read_log  # noqa: E402


class Missing(Exception):
    """Something a criterion needs is not available: a measurement, a mapping, a capture."""


def git(*a: str) -> str:
    return subprocess.run(['git', '-C', str(REPO), *a], capture_output=True, text=True).stdout.strip()


def repo_sha() -> str:
    return git('rev-parse', '--short=7', 'HEAD') + ('-dirty' if git('status', '--porcelain') else '')


def now_iso() -> str:
    return dt.datetime.now().isoformat(timespec='seconds')


def ts(iso: str | None) -> float | None:
    return dt.datetime.fromisoformat(iso).timestamp() if iso else None


def definitions() -> dict:
    return json.loads(CRITERIA.read_text())


def fmt(v: float) -> str:
    return f'{v:.4g}'


# Plain functions allowed in a criterion expression; every other NAME(...) is a scope measurement.
HELPERS = {'abs': abs, 'max': max, 'min': min}


def measurement_type(name: str) -> tuple[str, tuple | None]:
    """@brief (scope TYPe, edges) of a measurement name in an expression. DELAY_RISE_FALL(a,b) is the
    scope DELAY from a rising edge of a to the next falling edge of b; plain names have no edges."""
    m = re.fullmatch(r'DELAY_(RISE|FALL)_(RISE|FALL)', name.upper())
    return ('DELAY', (m.group(1), m.group(2))) if m else (name.upper(), None)


# ---- bench sheet --------------------------------------------------------------------------------
def sheet_sections(path: Path) -> dict:
    """@brief {test ID: markdown of its '## L2-XXX-nn · …' section} in sheet order."""
    out, cur, buf = {}, None, []
    if not path.exists():
        return out
    for line in path.read_text(encoding='utf-8').splitlines():
        m = re.match(r'## (L2-[A-Z]{3}-\d\d)\b', line)
        if line.startswith('## '):
            if cur:
                out[cur] = '\n'.join(buf)
            cur, buf = (m.group(1) if m else None), []
            continue
        if cur:
            buf.append(line)
    if cur:
        out[cur] = '\n'.join(buf)
    return out


def md_html(md: str) -> str:
    """@brief The small Markdown subset of the bench sheet (headings, lists, tables, bold, code, links) as HTML."""
    def inline(s):
        s = html.escape(s)
        s = re.sub(r'`([^`]+)`', r'<code>\1</code>', s)
        s = re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', s)
        s = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'\1', s)
        return s.replace('&lt;br&gt;', '<br>')
    out, lines, i = [], md.splitlines(), 0
    while i < len(lines):
        ln = lines[i]
        if not ln.strip():
            i += 1
            continue
        if ln.startswith('```'):
            j = i + 1
            while j < len(lines) and not lines[j].startswith('```'):
                j += 1
            out.append('<pre>' + html.escape('\n'.join(lines[i + 1:j])) + '</pre>')
            i = j + 1
            continue
        if ln.startswith('#'):
            out.append(f'<h4>{inline(ln.lstrip("#").strip())}</h4>')
            i += 1
            continue
        if ln.startswith('|'):
            rows = []
            while i < len(lines) and lines[i].startswith('|'):
                cells = [c.strip() for c in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?', c) for c in cells):
                    rows.append(cells)
                i += 1
            out.append('<table>' + ''.join('<tr>' + ''.join(f'<{"th" if k == 0 else "td"}>{inline(c)}</{"th" if k == 0 else "td"}>'
                                                            for c in r) + '</tr>' for k, r in enumerate(rows)) + '</table>')
            continue
        m = re.match(r'\s*(\d+\.|-)\s+(.*)', ln)
        if m:
            tag = 'ol' if m.group(1)[0].isdigit() else 'ul'
            items = []
            while i < len(lines):
                m = re.match(r'(\s*)(\d+\.|-)\s+(.*)', lines[i])
                if m:
                    items.append(inline(m.group(3)))
                elif lines[i].startswith('   ') and items:
                    items[-1] += ' ' + inline(lines[i].strip())
                else:
                    break
                i += 1
            out.append(f'<{tag}>' + ''.join(f'<li>{x}</li>' for x in items) + f'</{tag}>')
            continue
        para = [ln]
        i += 1
        while i < len(lines) and lines[i].strip() and not re.match(r'(#|\||```|\s*(\d+\.|-)\s)', lines[i]):
            para.append(lines[i])
            i += 1
        out.append('<p>' + inline(' '.join(para)) + '</p>')
    return '\n'.join(out)


# ---- session ------------------------------------------------------------------------------------
class Session:
    """@brief One bench session = one campaign folder. All writes go through here (one lock)."""

    def __init__(self, camp: Path):
        self.dir = camp
        self.lock = threading.RLock()
        self.defs = definitions()
        self.cat = load_catalogue()
        self.info = json.loads((camp / 'bench_session.json').read_text())
        self.info.setdefault('channels', {k: v for k, v in self.defs['default_channels'].items() if not k.startswith('_')})
        self.info.setdefault('app_measurements', [])
        self._pages_pending = threading.Event()
        self._pages_thread = None

    # -- creation and lookup
    @staticmethod
    def start(operator: str = '', firmware: str = '', note: str = '', dry_run: bool = False) -> 'Session':
        now = dt.datetime.now()
        sha = repo_sha()
        camp = OUT / f'{now:%F}_{sha}_{now:%H%M%S}_bench'
        camp.mkdir(parents=True)
        defs = definitions()
        info = {'started': now.isoformat(timespec='seconds'), 'operator': operator or git('config', 'user.name'),
                'firmware': firmware or f'{sha} (repository HEAD at the start; not read from the board)',
                'repo_commit': sha, 'note': note,
                'channels': {k: v for k, v in defs['default_channels'].items() if not k.startswith('_')},
                'app_measurements': []}
        (camp / 'bench_session.json').write_text(json.dumps(info, indent=1, ensure_ascii=False))
        if dry_run:  # shown in the Runs tab, never a test's current result
            (camp / 'reference.json').write_text(json.dumps({'reference': 'dry run of the L2 bench app, not a bench session'}))
        CURRENT.unlink(missing_ok=True)
        CURRENT.symlink_to(camp.name)
        return Session(camp)

    @staticmethod
    def current() -> 'Session | None':
        if CURRENT.is_symlink() and CURRENT.resolve().is_dir() and (CURRENT.resolve() / 'bench_session.json').exists():
            return Session(CURRENT.resolve())
        return None

    @property
    def dry_run(self) -> bool:
        return (self.dir / 'reference.json').exists()

    def save_info(self) -> None:
        with self.lock:
            (self.dir / 'bench_session.json').write_text(json.dumps(self.info, indent=1, ensure_ascii=False))

    # -- tests
    def test_ids(self) -> list[str]:
        """@brief Every L2 test of the catalogue, in bench-sheet order (pre-checks first), then the others."""
        l2 = [t['id'] for t in self.cat['tests'] if t['level'] == 'L2']
        order = []
        for t in l2:
            for sid in sheet_sections(self.sheet_path(t)):
                if sid in l2 and sid not in order:
                    order.append(sid)
        order += [t for t in l2 if t not in order]
        return order

    def spec(self, test: str) -> dict:
        return self.defs['tests'].get(test, {'points': [], 'keys': [], 'criteria': []})

    def sheet_path(self, test: str) -> Path:
        src = self.cat['by_id'].get(test, {}).get('source', '')
        m = re.search(r'docs/\S+\.md', src)
        return REPO / (m.group(0) if m else 'docs/testing/bench/inverter-session.md')

    def sheet_html(self, test: str) -> str:
        return md_html(sheet_sections(self.sheet_path(test)).get(test, '*No section for this test in the bench sheet.*'))

    def folder(self, test: str) -> Path:
        lvl, obj, nn = test.split('-')
        return self.dir / f'{lvl.lower()}_{obj.lower()}{nn}'

    def manifest(self, test: str) -> dict:
        p = self.folder(test) / 'bench_manifest.json'
        m = json.loads(p.read_text()) if p.exists() else {}
        m.setdefault('id', test)
        m.setdefault('criteria', [test])
        m.setdefault('points', {})
        m.setdefault('checks', {})
        m.setdefault('notes', [])
        m.setdefault('started', None)
        m.setdefault('ended', None)
        m.setdefault('firmware_captures', [])
        for name, pt in list(m['points'].items()):  # first format (one capture per point)
            if 'dir' in pt:
                m['points'][name] = {'captures': [pt], 'pinned': None}
        return m

    def save_manifest(self, test: str, man: dict) -> None:
        with self.lock:
            self.folder(test).mkdir(parents=True, exist_ok=True)
            (self.folder(test) / 'bench_manifest.json').write_text(json.dumps(man, indent=1, ensure_ascii=False))

    def touched(self) -> list[str]:
        return [t for t in self.test_ids() if (self.folder(t) / 'bench_manifest.json').exists()]

    # -- channels and measurements
    def channels(self, test: str | None = None) -> dict:
        """@brief Channel map of a step: the session's, with the step's own channels (criteria.json) on top."""
        out = dict(self.info['channels'])
        if test:
            out.update({k: v for k, v in self.spec(test).get('channels', {}).items() if not k.startswith('_')})
        return out

    def channel_of(self, name: str, test: str | None = None) -> str:
        if re.fullmatch(r'(CH|MATH|REF)\d+', name, re.I):
            return name.upper()
        for ch, c in self.channels(test).items():
            if c.get('signal') == name:
                return ch
        raise Missing(f'signal {name} is not mapped to a scope channel (session setup)')

    def measurement_plan(self) -> list[tuple]:
        """@brief Scope measurements the session needs: [(TYPE, src1, src2|None, edges|None, why)]."""
        plan = []
        std = self.defs.get('standard_measurements', {})
        for ch, c in sorted(self.info['channels'].items()):
            for typ in std.get(c.get('kind', ''), []):
                plan.append((typ, ch, None, None, f'standard on {c.get("signal") or ch}'))
        for test, sp in self.defs['tests'].items():
            if test.startswith('_'):
                continue
            for ch, c in sorted(sp.get('channels', {}).items()):
                for typ in std.get(c.get('kind', ''), []):
                    plan.append((typ, ch, None, None, f'standard on {c.get("signal") or ch} ({test})'))
            for c in sp['criteria']:
                for name, args in re.findall(r'([A-Za-z_]\w*)\s*\(([^()]*)\)', c.get('value', '')):
                    if name in HELPERS:
                        continue
                    srcs = [a.strip() for a in args.split(',') if a.strip()]
                    try:
                        chs = [self.channel_of(s, test) for s in srcs]
                    except Missing:
                        continue
                    typ, edges = measurement_type(name)
                    plan.append((typ, chs[0], chs[1] if len(chs) > 1 else None, edges, f'{test}{c["id"]}'))
        seen, out = set(), []
        for p in plan:
            if p[:4] not in seen:
                seen.add(p[:4])
                out.append(p)
        return out

    # -- results
    def evidence(self, man: dict, point: str) -> dict | None:
        pt = man['points'].get(point)
        if not pt or not pt['captures']:
            return None
        i = pt.get('pinned')
        return pt['captures'][i] if i is not None and i < len(pt['captures']) else pt['captures'][-1]

    def evaluate(self, expr: str, cap: dict, stat: str, test: str | None = None):
        """@brief Value of an expression over a capture's measurements. @return (value, [measurements used])."""
        used = []

        def fn(name):
            typ, edges = measurement_type(name)

            def f(*srcs):
                want = list(srcs) + [None] * (2 - len(srcs))
                for m in cap['measurements']:
                    if (m.get('type') or '').upper() != typ or (m.get('source1') or '').upper() != want[0]:
                        continue
                    if want[1] and (m.get('source2') or '').upper() != want[1]:
                        continue
                    if edges and tuple(m.get('edges') or ()) != edges:
                        continue
                    v = m[stat]['mean']
                    if v is None:
                        raise Missing(f'{name}({",".join(srcs)}) has no value (low amplitude or no acquisition)')
                    used.append(m)
                    return v
                raise Missing(f'scope measurement {name}({",".join(srcs)}) is not set up (Configure scope)')
            return f
        funcs = set(re.findall(r'([A-Za-z_]\w*)\s*\(', expr))
        ns = {}
        for name in set(re.findall(r'[A-Za-z_]\w*', expr)):
            ns[name] = (HELPERS[name] if name in HELPERS else fn(name) if name in funcs
                        else self.channel_of(name, test))
        return eval(expr, {'__builtins__': {}}, ns), used  # noqa: S307 (expressions come from criteria.json)

    def window(self, man: dict) -> tuple[float, float] | None:
        t0 = ts(man.get('started'))
        return (t0, ts(man.get('ended')) or time.time()) if t0 else None

    def telemetry_part(self, conds: list, man: dict, console=None) -> tuple[str, str]:
        """@brief (verdict, text) of a criterion's telemetry conditions over the step window."""
        w = self.window(man)
        if not w:
            return 'BLOCKED', 'step not started (Start step, or send a key from the step)'
        if console is not None:
            rx, keys = console.window(*w)
        else:
            rx_all, keys_all = read_log(self.dir / 'console.txt')
            rx = [(t, x) for t, x in rx_all if w[0] <= t <= w[1]]
            keys = [(t, k) for t, k in keys_all if w[0] <= t <= w[1]]
        tel_all = [(t, d) for t, x in rx for d in [parse_telemetry(x)] if d]
        rx_all = rx
        verdicts, texts = [], []
        clock = lambda t: dt.datetime.fromtimestamp(t).strftime('%H:%M:%S')  # noqa: E731
        for c in conds:
            # Optional bounds inside the step window: after the last 'after_key', before the first 'before_key'.
            lo_t, hi_t, bound = w[0], w[1], ''
            if 'after_key' in c:
                k = next((t for t, key in reversed(keys) if key == c['after_key']), None)
                if k is None:
                    verdicts.append('BLOCKED')
                    texts.append(f'key {c["after_key"]} not sent in the step')
                    continue
                lo_t, bound = k, f' after key {c["after_key"]}'
            if 'before_key' in c:
                k = next((t for t, key in keys if key == c['before_key'] and t >= lo_t), None)
                if k is not None:
                    hi_t, bound = k, bound + f' before key {c["before_key"]}'
            rx = [(t, x) for t, x in rx_all if lo_t <= t <= hi_t]
            tel = [(t, d) for t, d in tel_all if lo_t <= t <= hi_t]
            if 'console_has' in c or 'console_lacks' in c:
                s = c.get('console_has') or c.get('console_lacks')
                hit = next(((t, x) for t, x in rx if s.lower() in x.lower()), None)
                ok = bool(hit) == ('console_has' in c)
                verdicts.append('PASS' if ok else 'FAIL')
                texts.append(f'"{s}" {"printed at " + clock(hit[0]) if hit else "not printed"}{bound}')
                continue
            if 'time_from_key' in c:
                k = next((t for t, key in reversed(keys) if key == c['time_from_key']), None)
                reach = next((t for t, d in tel if k and t >= k and d.get(c['field']) == c['value']), None)
                verdicts.append('INFO')
                texts.append(f'{c["field"]} {c["value"]} {reach - k:.1f} s after key {c["time_from_key"]}' if reach
                             else f'{c["field"]} {c["value"]} not reached after key {c["time_from_key"]}' if k
                             else f'key {c["time_from_key"]} not sent in the step')
                continue
            f = c['field']
            vals = [(t, d[f]) for t, d in tel if f in d]
            if not vals:
                verdicts.append('BLOCKED')
                texts.append(f'no telemetry line in the step window ({f}{bound})')
                continue
            if 'reaches' in c:
                hit = next(((t, v) for t, v in vals if v == c['reaches']), None)
                verdicts.append('PASS' if hit else 'FAIL')
                texts.append(f'{f} {c["reaches"]} at {clock(hit[0])}{bound}' if hit else
                             f'{f} never {c["reaches"]} on {len(vals)} lines{bound}')
            elif 'all' in c:
                bad = [(t, v) for t, v in vals if v != c['all']]
                verdicts.append('FAIL' if bad else 'PASS')
                texts.append(f'{f} {c["all"]} on all {len(vals)} lines{bound}' if not bad else
                             f'{f} {bad[0][1]} at {clock(bad[0][0])} ({len(bad)} of {len(vals)} lines not {c["all"]}){bound}')
            elif 'final' in c:
                verdicts.append('PASS' if vals[-1][1] == c['final'] else 'FAIL')
                texts.append(f'{f} {vals[-1][1]} at {clock(vals[-1][0])}{bound}')
            elif 'holds' in c:
                i = next((k for k, (_, v) in enumerate(vals) if v == c['holds']), None)
                if i is None:
                    verdicts.append('FAIL')
                    texts.append(f'{f} never {c["holds"]} ({len(vals)} lines)')
                else:
                    bad = [(t, v) for t, v in vals[i:] if v != c['holds']]
                    verdicts.append('FAIL' if bad else 'PASS')
                    texts.append(f'{f} {c["holds"]} from {clock(vals[i][0])}, held on {len(vals) - i} lines' if not bad else
                                 f'{f} left {c["holds"]} at {clock(bad[0][0])} (to {bad[0][1]})')
            elif 'range' in c:
                lo, hi = c['range']
                nums = [float(v) for _, v in vals]
                ok = all(lo <= x <= hi for x in nums)
                verdicts.append('PASS' if ok else 'FAIL')
                texts.append(f'{f} {min(nums):.4g} to {max(nums):.4g} over {len(nums)} lines{bound}')
        v = ('FAIL' if 'FAIL' in verdicts else 'BLOCKED' if 'BLOCKED' in verdicts else
             'PASS' if 'PASS' in verdicts else 'INFO')
        return v, 'telemetry: ' + '; '.join(texts)

    def judge(self, test: str, man: dict | None = None, console=None) -> list[tuple]:
        """@brief Result rows (id, verdict, desc, limit, measured) of one test."""
        man = man or self.manifest(test)
        rows = []
        for c in self.spec(test)['criteria']:
            parts = []  # (verdict, text, required)
            lo, hi, unit = c.get('lo'), c.get('hi'), c.get('unit', '')
            limit = (f'{fmt(lo)} to {fmt(hi)} {unit}'.strip() if lo is not None and hi is not None
                     else c.get('limit', 'record'))
            if 'value' in c and 'points' in c:  # worst over several points (e.g. three relay operations)
                vals, missing = [], []
                for point in c['points']:
                    cap_ref = self.evidence(man, point)
                    if not cap_ref:
                        missing.append(f'{point} not captured')
                        continue
                    cap = json.loads((self.folder(test) / cap_ref['dir'] / 'capture.json').read_text())
                    try:
                        vals.append((point, self.evaluate(c['value'], cap, cap_ref['stats'], test)[0]))
                    except Missing as ex:
                        missing.append(f'{point}: {ex}')
                agg = c.get('agg', 'max')
                each = ', '.join(f'{p} {fmt(x)}' for p, x in vals)
                if missing:
                    parts.append(('BLOCKED', '; '.join(missing) + (f' (so far: {each})' if vals else ''), True))
                else:
                    pick = {'max': max, 'min': min, 'maxabs': lambda xs: max(xs, key=abs)}[agg]
                    v = pick([x for _, x in vals])
                    meas = f'{agg} {fmt(v)} {unit}'.strip() + f' over {each}'
                    parts.append(('INFO' if lo is None or hi is None else 'PASS' if lo <= v <= hi else 'FAIL', meas, True))
            elif 'value' in c:
                point = c.get('point') or (self.spec(test)['points'] or ['main'])[0]
                cap_ref = self.evidence(man, point)
                if not cap_ref:
                    parts.append(('BLOCKED', f'point {point} not captured yet', True))
                else:
                    cap = json.loads((self.folder(test) / cap_ref['dir'] / 'capture.json').read_text())
                    try:
                        v, used = self.evaluate(c['value'], cap, cap_ref['stats'], test)
                        meas = f'{fmt(v)} {unit}'.strip()
                        if len(used) == 1 and cap_ref['stats'] == 'all':
                            a = used[0]['all']
                            if a.get('minimu') is not None and a.get('maximu') is not None:
                                meas += f' (min {fmt(a["minimu"])}, max {fmt(a["maximu"])}, n {a.get("popula") or 0:.0f})'
                        meas += f'; point {point}, {cap_ref["timestamp"][11:]}'
                        parts.append(('INFO' if lo is None or hi is None else 'PASS' if lo <= v <= hi else 'FAIL', meas, True))
                    except Missing as ex:
                        parts.append(('BLOCKED', f'{ex} (point {point})', True))
            if 'telemetry' in c:
                v, text = self.telemetry_part(c['telemetry'], man, console)
                parts.append((v, text, True))
            if c.get('operator'):
                a = man['checks'].get(c['id'])
                info = c['operator'] == 'info'
                if a:
                    text = a.get('text') or {'PASS': 'yes', 'FAIL': 'no'}.get(a['verdict'], '')
                    parts.append(('INFO' if info else a['verdict'], f'operator: {text}', True))
                else:
                    parts.append(('BLOCKED', 'operator check not answered', not (info and len(parts) > 0)))
            verdicts = [p[0] for p in parts if p[2]]
            v = ('FAIL' if 'FAIL' in verdicts else 'BLOCKED' if 'BLOCKED' in verdicts or not verdicts else
                 'PASS' if 'PASS' in verdicts else 'INFO')
            rows.append((test + c['id'], v, c['desc'], limit, ' · '.join(p[1] for p in parts if p[2] or p[0] != 'BLOCKED')))
        return rows

    def write_log(self, test: str, console=None) -> list[tuple]:
        rows = self.judge(test, console=console)
        clean = lambda s: str(s).replace('\t', ' ').replace('\n', ' ')  # noqa: E731
        with self.lock:
            (self.dir / f'{self.folder(test).name}.log').write_text(
                ''.join('@@\t' + '\t'.join(clean(x) for x in r) + '\n' for r in rows))
        return rows

    # -- pages (record, review page, control center), coalesced in a background thread
    def regenerate_pages(self, wait: bool = False) -> int:
        if wait:
            return self._regenerate()
        self._pages_pending.set()
        if not (self._pages_thread and self._pages_thread.is_alive()):
            self._pages_thread = threading.Thread(target=self._pages_loop, daemon=True)
            self._pages_thread.start()
        return 0

    def _pages_loop(self) -> None:
        while self._pages_pending.is_set():
            self._pages_pending.clear()
            self._regenerate()

    def _regenerate(self) -> int:
        py = str(REPO / '.venv/bin/python') if (REPO / '.venv/bin/python').exists() else sys.executable
        logs = sorted(str(p) for p in self.dir.glob('*.log'))
        if not logs:
            return 0
        rc = subprocess.run([sys.executable, str(REPO / 'tests/common/make_record.py'), '--out', str(self.dir),
                             '--levels', 'L2', *logs], capture_output=True, text=True).returncode
        subprocess.run([py, str(REPO / 'tests/common/review_report.py'), str(self.dir)], capture_output=True)
        subprocess.run([sys.executable, str(REPO / 'tests/common/control_center.py'), '--out', str(OUT)],
                       capture_output=True)
        return rc


# ---- capture (shared by the app and bench.py) ---------------------------------------------------
def firmware_settings(tel: dict | None) -> dict:
    """@brief The settings the board reports in its telemetry (feedforward source, current PI gains), or {}."""
    if not tel:
        return {}
    out = {}
    if tel.get('ff') is not None: out['ff'] = int(tel['ff'])
    if tel.get('kp') is not None: out['kp'] = float(tel['kp'])
    if tel.get('ti') is not None: out['ti_ms'] = float(tel['ti'])
    if tel.get('dt') is not None: out['dt_ns'] = int(tel['dt'])
    if tel.get('ffn') is not None: out['ffn'] = int(tel['ffn'])
    if tel.get('dcl') is not None: out['dcl'] = int(tel['dcl'])
    if tel.get('vdc') is not None: out['vdc'] = float(tel['vdc'])
    return out


def settings_text(s: dict) -> str:
    """@brief 'Kp 3.5 V/A, Ti 20 ms, feedforward measured' for a firmware_settings() dict ('' when empty)."""
    parts = []
    if 'kp' in s: parts.append(f'Kp {s["kp"]:g} V/A')
    if 'ti_ms' in s: parts.append(f'Ti {s["ti_ms"]:g} ms')
    if 'ff' in s: parts.append('feedforward ' + ('SOGI' if s['ff'] else 'measured'))
    if 'dt_ns' in s: parts.append(f'dead time {s["dt_ns"]} ns')
    if 'ffn' in s: parts.append('feedforward filter ' + ('on' if s['ffn'] else 'off'))
    if 'dcl' in s: parts.append('DC loop ' + ('on' if s['dcl'] else 'off') + (f' ({s["vdc"]:+.2f} V)' if s.get('dcl') and 'vdc' in s else ''))
    return ', '.join(parts)


def record_capture(sess: Session, sc, test: str, point: str, seconds: float = 2.0, waveforms: bool = False,
                   single: bool = False, note: str = '', settings: dict | None = None) -> dict:
    """@brief Capture the scope for one point of a test and add it to the test's history.
    A running scope has its statistics restarted and collected for `seconds`; after a single shot (or a
    stopped scope) the acquisition on screen is read as it is. @return the capture entry of the manifest."""
    from scope import capture
    stats_note = ('single shot: the triggered acquisition' if single else sc.settle(seconds))
    stamp = dt.datetime.now()
    rel = f'captures/{stamp:%H%M%S}_{point}'
    cap = capture(sc, sess.folder(test) / rel, label=f'{test} {point}', waveforms=waveforms, stats_note=stats_note)
    entry = {'dir': rel, 'timestamp': cap['timestamp'],
             'stats': 'all' if stats_note.startswith('statistics restarted') else 'current',
             'stats_note': stats_note, 'waveforms': waveforms, 'note': note, 'settings': settings or {}}
    with sess.lock:
        man = sess.manifest(test)
        pt = man['points'].setdefault(point, {'captures': [], 'pinned': None})
        pt['captures'].append(entry)
        if man['started'] is None:
            man['started'] = cap['timestamp']
        sess.save_manifest(test, man)
    return entry
