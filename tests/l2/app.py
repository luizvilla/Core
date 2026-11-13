#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
## @file app.py
## @brief L2 bench app: a local web page that walks the tester through the bench sheet, sends menu keys,
##        logs the console, captures the oscilloscope and records operator answers into the session campaign.
## @par Usage (repo root)
##   .venv/bin/python tests/l2/app.py                 # open session (or a new one), http://localhost:8765
##   .venv/bin/python tests/l2/app.py --new --note "400 V / 0.5 A, 100 W bulb"
##   .venv/bin/python tests/l2/app.py --serial /dev/ttyACM0 --port 8800 --no-browser
## @note Listens on 127.0.0.1 only: the page can send menu keys to the board. One app per session: it owns
##   the serial port (close any pio monitor first) and the scope.
"""Local HTTP server for bench sessions: JSON API + one page (tests/l2/web/index.html)."""
from __future__ import annotations

import argparse
import glob
import json
import mimetypes
import re
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from session import REPO, OUT, Session, firmware_settings, now_iso, record_capture
from console import Console, INV_STATES, STAGES, decode_dump, fw_timebase, parse_telemetry

WEB = Path(__file__).resolve().parent / 'web'


class Bench:
    """@brief Server state: the session, the console, the scope and the background job."""

    def __init__(self, sess: Session, serial_port: str | None, resource: str | None, use_scope: bool):
        self.resource, self.use_scope, self.serial_port = resource, use_scope, serial_port
        self.scope = None
        self.scope_error = ''
        self.scope_lock = threading.Lock()
        self.job = {'busy': False, 'text': '', 'error': '', 'done': ''}
        self.cancel = threading.Event()
        self.plan_status = []
        self.active = None
        self.open(sess)

    def open(self, sess: Session) -> None:
        self.sess = sess
        sess.info['serial_port'] = self.serial_port or ''
        sess.save_info()
        old = getattr(self, 'console', None)
        if old is not None and old.ser is not None:
            old._stop.set()
            try:
                old.ser.close()
            except Exception:  # noqa: BLE001
                pass
        self.console = Console(self.serial_port, sess.dir / 'console.txt', sess.dir / 'firmware_scope', self.on_dump)

    # -- scope
    def get_scope(self):
        if not self.use_scope:
            raise RuntimeError('scope disabled (--no-scope)')
        if self.scope is None:
            from scope import Scope
            try:
                self.scope = Scope(self.resource)
                self.scope_error = ''
            except SystemExit as ex:  # Scope exits when no instrument is found
                self.scope_error = str(ex)
                raise RuntimeError(self.scope_error) from None
        return self.scope

    def run_job(self, text: str, fn) -> None:
        if self.job['busy']:
            raise RuntimeError(f'busy: {self.job["text"]}')
        self.job = {'busy': True, 'text': text, 'error': '', 'done': ''}
        self.cancel.clear()

        def work():
            try:
                with self.scope_lock:
                    self.job['done'] = fn() or 'done'
            except Exception as ex:  # noqa: BLE001  (shown on the page)
                self.job['error'] = f'{ex}'
                traceback.print_exc()
                if 'Timeout' in type(ex).__name__ or 'VI_ERROR' in str(ex):
                    self.scope = None  # reconnect on the next use
            finally:
                self.job['busy'] = False
        threading.Thread(target=work, daemon=True).start()

    # -- results
    def changed(self, test: str) -> None:
        self.sess.write_log(test, self.console)
        self.sess.regenerate_pages()

    def on_dump(self, t: float, txt: Path, csv: Path | None) -> None:
        test = self.active
        if not test:
            return
        with self.sess.lock:
            man = self.sess.manifest(test)
            rate, trig, why = fw_timebase(self.console.rx, t)
            man['firmware_captures'].append({'time': now_iso(), 'txt': str(txt.relative_to(self.sess.dir)),
                                             'csv': str(csv.relative_to(self.sess.dir)) if csv else None,
                                             'rate_hz': rate, 'rate_why': why, 'trigger_index': trig,
                                             'settings': firmware_settings(self.console.latest())})
            self.sess.save_manifest(test, man)
        self.changed(test)

    # -- views
    def state(self) -> dict:
        s = self.sess
        tests = []
        for tid in s.test_ids():
            cat = s.cat['by_id'].get(tid, {})
            counts = {}
            if (s.folder(tid) / 'bench_manifest.json').exists():
                for r in s.judge(tid, console=self.console):
                    counts[r[1]] = counts.get(r[1], 0) + 1
            tests.append({'id': tid, 'title': cat.get('title', ''), 'defined': bool(s.spec(tid)['criteria']),
                          'counts': counts})
        tel = self.console.latest()
        if tel:
            tel['stage_name'] = STAGES.get(tel['st'], '?')
            tel['inv_name'] = INV_STATES.get(tel['inv'], '?')
            tel['age'] = round(time.time() - tel['t'], 1)
        return {'session': {'name': s.dir.name, 'dry_run': s.dry_run, **s.info},
                'tests': tests, 'active': self.active,
                'console': [{'t': time.strftime('%H:%M:%S', time.localtime(t)), 'dir': d, 'text': x}
                            for t, d, x in self.console.tail(150)],
                'telemetry': tel,
                'serial': {'port': self.serial_port, 'connected': self.console.connected, 'error': self.console.error},
                'scope': {'connected': self.scope is not None, 'error': self.scope_error, 'enabled': self.use_scope},
                'job': self.job, 'plan': self.plan_status, 'fwscope': self.fw_list(),
                'keys': {k: v for k, v in s.defs['keys'].items() if not k.startswith('_')}}

    def test_view(self, tid: str, sheet: bool) -> dict:
        s = self.sess
        man = s.manifest(tid)
        spec = s.spec(tid)
        cat = s.cat['by_id'].get(tid, {})
        points = {}
        for name, pt in man['points'].items():
            caps = []
            for i, c in enumerate(pt['captures']):
                d = s.folder(tid) / c['dir']
                cj = json.loads((d / 'capture.json').read_text()) if (d / 'capture.json').exists() else {}
                caps.append({**c, 'index': i, 'screen': f'/out/{s.folder(tid).name}/{c["dir"]}/screen.png',
                             'has_waveforms': (d / 'waveforms.npz').exists(),
                             'measurements': [{'id': m['id'], 'type': m['type'], 'source1': m['source1'],
                                               'source2': m.get('source2'), 'edges': m.get('edges'), 'unit': m.get('unit'),
                                               **{k: m[c['stats']].get(k) for k in ('mean', 'minimu', 'maximu', 'popula')}}
                                              for m in cj.get('measurements', [])]})
            points[name] = {'captures': caps, 'pinned': pt.get('pinned')}
        rows = s.judge(tid, man, console=self.console)
        return {'id': tid, 'title': cat.get('title', ''), 'criterion_text': cat.get('criterion', ''),
                'conditions': cat.get('conditions', ''), 'spec': spec, 'manifest': man, 'points': points,
                'channels': s.channels(tid),
                'rows': [dict(zip(('id', 'verdict', 'desc', 'limit', 'measured'), r)) for r in rows],
                'sheet': s.sheet_html(tid) if sheet else None,
                'report': f'/out/report.html#{s.folder(tid).name}'}

    # -- firmware scope (key r): dumps decoded to CSV by console.py
    def fw_list(self) -> list:
        """@brief Firmware scope CSVs of the session, newest first, with the test they were taken in."""
        d = self.sess.dir / 'firmware_scope'
        owner = {}
        for t in self.sess.touched():
            for f in self.sess.manifest(t)['firmware_captures']:
                if f.get('csv'):
                    owner[Path(f['csv']).name] = t
        out = []
        for csv in sorted(d.glob('*.csv'), reverse=True) if d.is_dir() else []:
            stem = csv.stem.split('_')[0]
            out.append({'name': csv.name, 'time': f'{stem[:2]}:{stem[2:4]}:{stem[4:6]}', 'test': owner.get(csv.name),
                        'url': f'/out/firmware_scope/{csv.name}'})
        return out

    def fw_rate(self, csv: Path) -> tuple[int, int, str]:
        """@brief (rate, trigger sample, why) of a dump: as recorded when it arrived, else from the console."""
        for t in self.sess.touched():
            for f in self.sess.manifest(t)['firmware_captures']:
                if f.get('csv') and Path(f['csv']).name == csv.name and f.get('rate_hz') and f.get('trigger_index'):
                    return f['rate_hz'], f['trigger_index'], f.get('rate_why', 'recorded with the dump')
        import datetime as dt
        stem = csv.stem.split('_')[0]
        day = dt.datetime.fromisoformat(self.sess.info['started']).date()
        t_dump = dt.datetime.combine(day, dt.time(int(stem[:2]), int(stem[2:4]), int(stem[4:6]))).timestamp() + 1
        return fw_timebase(self.console.rx, t_dump)

    def fw_data(self, name: str) -> dict:
        csv = (self.sess.dir / 'firmware_scope' / name).resolve()
        if not csv.is_relative_to((self.sess.dir / 'firmware_scope').resolve()) or not csv.is_file():
            raise FileNotFoundError(name)
        txt = csv.with_suffix('.txt')
        if txt.exists():  # re-decode from the raw dump: CSVs written before 2026-11-13 were not rotated ('## n')
            decode_dump(txt.read_text().splitlines(), csv)
        lines = csv.read_text().splitlines()
        names = [n.strip() for n in lines[0].split(',')]
        cols = [[] for _ in names]
        for ln in lines[1:]:
            vals = ln.split(',')
            if len(vals) != len(names):
                continue
            for c, v in zip(cols, vals):
                try:
                    c.append(float(v))
                except ValueError:
                    c.append(None)
        rate, trig, why = self.fw_rate(csv)
        return {'name': name, 'names': names, 'columns': cols, 'rate': rate, 'rate_why': why,
                'trigger': trig, 'url': f'/out/firmware_scope/{name}'}

    # -- oscilloscope waveforms (waveforms.npz of a capture), sent as a min/max envelope of the visible range
    def wave(self, tid: str, rel: str, t0: float | None, t1: float | None, n: int = 2000) -> dict:
        import numpy as np
        base = self.sess.folder(tid).resolve()
        d = (base / rel).resolve()
        if not d.is_relative_to(base) or not (d / 'waveforms.npz').is_file():
            raise FileNotFoundError(f'{tid} {rel}: no waveforms stored')
        cache = self.__dict__.setdefault('_wave_cache', {})
        key = str(d)
        if key not in cache:
            z = np.load(d / 'waveforms.npz')
            cache[key] = {k[:-2]: (z[k], z[k[:-2] + '_v']) for k in z.files if k.endswith('_t') and k[:-2] + '_v' in z.files}
            while len(cache) > 3:
                cache.pop(next(iter(cache)))
        chans = cache[key]
        cap = json.loads((d / 'capture.json').read_text()) if (d / 'capture.json').exists() else {}
        setup = cap.get('setup', {}).get('channels', {})
        sig = self.sess.channels(tid)
        out = {'names': [], 'units': [], 't': None, 'lo': [], 'hi': [], 'full': None, 'points': 0, 'bucket': 1}
        tmin = min(float(t[0]) for t, _ in chans.values())
        tmax = max(float(t[-1]) for t, _ in chans.values())
        out['full'] = [tmin * 1e3, tmax * 1e3]
        a = tmin if t0 is None else t0 / 1e3
        b = tmax if t1 is None else t1 / 1e3
        for ch in sorted(chans):
            t, v = chans[ch]
            i0, i1 = int(np.searchsorted(t, a, 'left')), int(np.searchsorted(t, b, 'right'))
            i0, i1 = max(0, i0 - 1), min(len(t), i1 + 1)
            seg_t, seg_v = t[i0:i1], v[i0:i1]
            k = max(1, int(np.ceil(len(seg_v) / n)))
            m = len(seg_v) // k
            if k > 1 and m > 0:
                blk = seg_v[:m * k].reshape(m, k)
                lo, hi, tt = blk.min(1), blk.max(1), seg_t[:m * k].reshape(m, k).mean(1)
            else:
                lo = hi = seg_v
                tt = seg_t
            if out['t'] is None:  # every channel of one acquisition shares the time base
                out['t'] = np.round(tt * 1e3, 9).tolist()
                out['points'], out['bucket'] = int(len(t)), k
            label = sig.get(ch, {}).get('signal') or ''
            out['names'].append(f'{ch} {label}'.strip())
            out['units'].append((setup.get(ch, {}) or {}).get('probe_units') or 'V')
            out['lo'].append(np.asarray(lo, dtype=np.float32).tolist())
            out['hi'].append(np.asarray(hi, dtype=np.float32).tolist())
        return out

    # -- actions
    def act(self, path: str, b: dict):
        s = self.sess
        tid = b.get('test')
        if path == '/api/active':
            self.active = tid
            return {}
        if path == '/api/key':
            key = b['key']
            if key not in s.defs['keys']:
                raise ValueError(f'key {key!r} is not in criteria.json "keys"')
            if tid:
                with s.lock:
                    man = s.manifest(tid)
                    if man['started'] is None or man['ended']:  # a key opens (or reopens) the step window
                        man['started'] = man['started'] or now_iso()
                        man['ended'] = None
                        s.save_manifest(tid, man)
            self.console.send(key)
            return {}
        if path == '/api/step':
            with s.lock:
                man = s.manifest(tid)
                if b['action'] == 'start':
                    man['started'], man['ended'] = now_iso(), None
                elif b['action'] == 'finish':
                    man['started'] = man['started'] or now_iso()
                    man['ended'] = now_iso()
                elif b['action'] == 'reopen':
                    man['ended'] = None
                s.save_manifest(tid, man)
            self.changed(tid)
            return {}
        if path == '/api/check':
            with s.lock:
                man = s.manifest(tid)
                if b.get('verdict') in (None, ''):
                    man['checks'].pop(b['id'], None)
                else:
                    man['checks'][b['id']] = {'verdict': b['verdict'], 'text': b.get('text', ''), 'time': now_iso()}
                s.save_manifest(tid, man)
            self.changed(tid)
            return {}
        if path == '/api/note':
            with s.lock:
                man = s.manifest(tid)
                man['notes'].append({'time': now_iso(), 'point': b.get('point', ''), 'text': b['text']})
                s.save_manifest(tid, man)
            self.changed(tid)
            return {}
        if path == '/api/pin':
            with s.lock:
                man = s.manifest(tid)
                man['points'][b['point']]['pinned'] = b.get('index')
                s.save_manifest(tid, man)
            self.changed(tid)
            return {}
        if path == '/api/acquire':
            point, single = b.get('point') or 'main', bool(b.get('single'))

            def job():
                sc = self.get_scope()
                if single:
                    sc.arm_single()
                    self.job['text'] = f'armed: waiting for the trigger ({tid} {point})'
                    t0 = time.time()
                    while sc.running():
                        if self.cancel.is_set():
                            sc.run_continuous()
                            return 'cancelled'
                        if time.time() - t0 > float(b.get('timeout', 300)):
                            raise RuntimeError('no trigger within the timeout')
                        time.sleep(0.3)
                    self.job['text'] = f'triggered: capturing {tid} {point}'
                entry = record_capture(s, sc, tid, point, float(b.get('seconds', 2)), bool(b.get('waveforms')) or single,
                                       settings=firmware_settings(self.console.latest()),
                                       single=single, note=b.get('note', ''))
                self.changed(tid)
                return f'captured {point} at {entry["timestamp"][11:]}'
            self.run_job(f'{"single shot" if single else "acquiring"} {tid} {point}', job)
            return {}
        if path == '/api/cancel':
            self.cancel.set()
            return {}
        if path == '/api/run':
            self.run_job('scope: run continuous', lambda: (self.get_scope().run_continuous(), 'scope running')[1])
            return {}
        if path == '/api/session':
            for k in ('operator', 'firmware', 'note'):
                if k in b:
                    s.info[k] = b[k]
            if 'channels' in b:
                s.info['channels'] = b['channels']
            s.save_info()
            for t in s.touched():
                s.write_log(t, self.console)
            s.regenerate_pages()
            return {}
        if path == '/api/configure':
            self.run_job('configuring scope measurements', self.configure)
            return {}
        if path == '/api/unconfigure':
            self.run_job('removing the measurements the app created', self.unconfigure)
            return {}
        if path == '/api/new':
            self.open(Session.start(b.get('operator', ''), b.get('firmware', ''), b.get('note', ''),
                                    bool(b.get('dry_run'))))
            self.active, self.plan_status = None, []
            return {}
        raise ValueError(f'unknown action {path}')

    def configure(self) -> str:
        sc = self.get_scope()
        existing = sc.measurement_list()
        key = lambda m: ((m['type'] or '').upper(), (m['source1'] or '').upper(), (m.get('source2') or None),  # noqa: E731
                         tuple(m['edges']) if m.get('edges') else None)
        have = {key(m): m['id'] for m in existing}
        status, created = [], 0
        for typ, s1, s2, edges, why in self.sess.measurement_plan():
            k = (typ, s1, s2, edges)
            label = typ + (f' {edges[0].lower()}→{edges[1].lower()}' if edges else '')
            src = f'{s1},{s2}' if s2 else s1
            if k in have:
                status.append({'type': label, 'src': src, 'id': have[k], 'state': 'on the scope', 'why': why})
                continue
            m = sc.add_measurement(typ, s1, s2, edges)
            if m:
                created += 1
                have[k] = m
                self.sess.info['app_measurements'].append({'id': m, 'type': typ, 'source1': s1, 'source2': s2,
                                                           'edges': list(edges) if edges else None})
            status.append({'type': label, 'src': src, 'id': m or '—',
                           'state': 'created' if m else 'scope refused', 'why': why})
        self.sess.save_info()
        self.plan_status = status
        return f'{created} measurements created, {len(status) - created} already there'

    def unconfigure(self) -> str:
        sc = self.get_scope()
        now = {m['id']: m for m in sc.measurement_list()}
        n = 0
        for m in self.sess.info['app_measurements']:
            cur = now.get(m['id'])
            if cur and (cur['type'] or '').upper() == m['type'] and (cur['source1'] or '').upper() == m['source1']:
                sc.delete_measurement(m['id'])
                n += 1
        self.sess.info['app_measurements'] = []
        self.sess.save_info()
        self.plan_status = []
        return f'{n} measurements removed'


def handler(bench: Bench):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'max-age=3600' if '/captures/' in self.path else 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def file(self, base: Path, rel: str):
            p = (base / rel).resolve()
            if not p.is_relative_to(base.resolve()) or not p.is_file():
                return self.send(404, {'error': 'not found'})
            self.send(200, p.read_bytes(), mimetypes.guess_type(p.name)[0] or 'application/octet-stream')

        def do_GET(self):
            u = urlparse(self.path)
            path = unquote(u.path)
            try:
                if path in ('/', '/index.html'):
                    return self.file(WEB, 'index.html')
                if path == '/api/state':
                    return self.send(200, bench.state())
                m = re.fullmatch(r'/api/test/(L2-[A-Z]{3}-\d\d)', path)
                if m:
                    return self.send(200, bench.test_view(m.group(1), 'sheet=1' in (u.query or '')))
                m = re.fullmatch(r'/api/wave/(L2-[A-Z]{3}-\d\d)/(captures/[\w.-]+)', path)
                if m:
                    from urllib.parse import parse_qs
                    q = {k: v[0] for k, v in parse_qs(u.query or '').items()}
                    f = lambda k: float(q[k]) if q.get(k) not in (None, '') else None  # noqa: E731
                    return self.send(200, bench.wave(m.group(1), m.group(2), f('t0'), f('t1'), int(q.get('n') or 2000)))
                m = re.fullmatch(r'/api/fwscope/([\w.-]+\.csv)', path)
                if m:
                    return self.send(200, bench.fw_data(m.group(1)))
                if path == '/control-center':
                    return self.file(OUT, 'control-center.html')
                if path.startswith('/out/'):
                    return self.file(bench.sess.dir, path[5:])
                if path.startswith('/tests/out/'):  # links inside the control center
                    return self.file(OUT, path[len('/tests/out/'):])
                return self.file(OUT, path.lstrip('/'))  # relative links from control-center.html
            except Exception as ex:  # noqa: BLE001
                traceback.print_exc()
                return self.send(500, {'error': str(ex)})

        def do_POST(self):
            if self.headers.get('Origin') not in (None, f'http://{self.headers.get("Host")}'):
                return self.send(403, {'error': 'cross-origin request refused'})
            n = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(n) or b'{}')
                return self.send(200, bench.act(urlparse(self.path).path, body) or {})
            except Exception as ex:  # noqa: BLE001  (shown on the page)
                return self.send(400, {'error': str(ex)})
    return H


def find_spin() -> str | None:
    found = sorted(glob.glob('/dev/serial/by-id/*SPIN*'))
    return found[0] if found else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--new', action='store_true', help='start a new session instead of the open one')
    ap.add_argument('--dry-run', action='store_true', help='with --new: a reference session, never a test result')
    ap.add_argument('--note', default='', help='with --new: session note')
    ap.add_argument('--firmware', default='', help='with --new: firmware on the board')
    ap.add_argument('--serial', help='serial port of the Spin (default: /dev/serial/by-id/*SPIN*)')
    ap.add_argument('--no-serial', action='store_true')
    ap.add_argument('--resource', help='VISA resource of the scope (default: the Tektronix on USB)')
    ap.add_argument('--no-scope', action='store_true')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--no-browser', action='store_true')
    a = ap.parse_args()
    sess = None if a.new else Session.current()
    sess = sess or Session.start(firmware=a.firmware, note=a.note, dry_run=a.dry_run)
    port = None if a.no_serial else (a.serial or find_spin())
    bench = Bench(sess, port, a.resource, not a.no_scope)
    srv = ThreadingHTTPServer(('127.0.0.1', a.port), handler(bench))
    url = f'http://localhost:{a.port}/'
    print(f'Bench session {sess.dir.relative_to(REPO)}{" (DRY RUN)" if sess.dry_run else ""}\n'
          f'Serial {port or "none"} · open {url} · Ctrl+C to stop')
    if not a.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
