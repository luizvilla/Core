#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
## @file bench.py
## @brief Command line for L2 bench sessions, for scripts and agents; the tester uses the app (app.py).
## @par Usage (repo root)
##   .venv/bin/python tests/l2/bench.py start [--firmware SHA] [--note TEXT] [--dry-run]
##   .venv/bin/python tests/l2/bench.py capture L2-INV-00 --point 400V [--waveforms] [--single]
##   .venv/bin/python tests/l2/bench.py check L2-INV-10 --check .a=pass --check ".c=fail:limit at 0.3 A"
##   .venv/bin/python tests/l2/bench.py step L2-INV-01 start|finish
##   .venv/bin/python tests/l2/bench.py status
## @note Same session files as the app (tests/l2/session.py). Do not capture from here while the app is
##   capturing: both would talk to the scope. Telemetry criteria read console.txt, written by the app.
## @return 0, or 1 when a criterion of the session FAILs (as make_record.py).
"""Bench-session command line: start, capture, operator checks, step window, status."""
from __future__ import annotations

import argparse
import re
import sys

from session import Session, now_iso, record_capture


def session(args) -> Session:
    s = Session.current() or Session.start(note=getattr(args, 'note', '') or '')
    if s.dry_run:
        print(f'*** {s.dir.name} is a DRY RUN: its results never count. Real session: bench.py start ***')
    return s


def apply_checks(s: Session, test: str, checks: list[str]) -> None:
    ops = {c['id']: c for c in s.spec(test)['criteria'] if c.get('operator')}
    with s.lock:
        man = s.manifest(test)
        for item in checks or []:
            m = re.fullmatch(r'(\.[\w-]+)=(pass|fail|info|yes|no)(?::(.*))?', item.strip(), re.I | re.S)
            if not m or m.group(1) not in ops:
                sys.exit(f'--check {item!r}: expected .x=pass|fail|info[:text] for one of {", ".join(ops) or "(none)"}')
            v = {'pass': 'PASS', 'yes': 'PASS', 'fail': 'FAIL', 'no': 'FAIL', 'info': 'INFO'}[m.group(2).lower()]
            man['checks'][m.group(1)] = {'verdict': 'INFO' if ops[m.group(1)]['operator'] == 'info' else v,
                                         'text': (m.group(3) or '').strip(), 'time': now_iso()}
        s.save_manifest(test, man)


def finish(s: Session, test: str) -> int:
    for r in s.write_log(test):
        print(f'{r[1]:8} {r[0]:16} {r[4]}')
    rc = s.regenerate_pages(wait=True)
    print(f'Review page: {s.dir.name}/report.html · control center: tests/out/control-center.html')
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('start')
    p.add_argument('--firmware', default='')
    p.add_argument('--operator', default='')
    p.add_argument('--note', default='')
    p.add_argument('--dry-run', action='store_true')
    p = sub.add_parser('capture')
    p.add_argument('test')
    p.add_argument('--point')
    p.add_argument('--seconds', type=float, default=2.0)
    p.add_argument('--waveforms', action='store_true')
    p.add_argument('--single', action='store_true', help='read the stopped scope as it is (after a single shot)')
    p.add_argument('--check', action='append')
    p.add_argument('--note', default='')
    p.add_argument('--resource')
    p = sub.add_parser('check')
    p.add_argument('test')
    p.add_argument('--check', action='append', required=True)
    p = sub.add_parser('step')
    p.add_argument('test')
    p.add_argument('action', choices=['start', 'finish', 'reopen'])
    sub.add_parser('status')
    a = ap.parse_args()

    if a.cmd == 'start':
        s = Session.start(a.operator, a.firmware, a.note, a.dry_run)
        print(f'Bench session: {s.dir.name}')
        return 0
    if a.cmd == 'status':
        s = Session.current()
        if not s:
            print('No bench session.')
            return 0
        print(f'{s.dir.name}{" (DRY RUN)" if s.dry_run else ""}')
        for t in s.touched():
            for r in s.judge(t):
                print(f'{r[1]:8} {r[0]:16} {r[4]}')
        return 0
    s = session(a)
    if a.test not in s.test_ids():
        sys.exit(f'{a.test}: not an L2 test of the catalogue ({", ".join(s.test_ids())})')
    if a.cmd == 'capture':
        pts = s.spec(a.test)['points']
        point = a.point or (pts[0] if len(pts) == 1 else 'main')
        if pts and point not in pts:
            sys.exit(f'{a.test}: --point must be one of {", ".join(pts)}')
        from scope import Scope
        entry = record_capture(s, Scope(a.resource), a.test, point, a.seconds, a.waveforms, a.single, a.note)
        print(f'Captured {a.test} {point} at {entry["timestamp"]} ({entry["stats_note"]})')
        apply_checks(s, a.test, a.check)
    elif a.cmd == 'check':
        apply_checks(s, a.test, a.check)
    elif a.cmd == 'step':
        with s.lock:
            man = s.manifest(a.test)
            if a.action == 'start':
                man['started'], man['ended'] = now_iso(), None
            elif a.action == 'finish':
                man['started'] = man['started'] or now_iso()
                man['ended'] = now_iso()
            else:
                man['ended'] = None
            s.save_manifest(a.test, man)
    return finish(s, a.test)


if __name__ == '__main__':
    sys.exit(main())
