#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file connection_l1.py
## @brief Switched-model validation (L1-GFL-14) of the connection strategy found by the design tests D-GFL-04/06.
## @par Usage (from the repository root)
##   python3 tests/l1/design/connection_l1.py build            # study builds (a minute)
##   python3 tests/l1/design/connection_l1.py run [--jobs N]   # the six runs, N in parallel (default: all)
##   python3 tests/l1/design/connection_l1.py report           # docs/testing/design/L1-GFL-14_IEC/report.md
## @par Builds (in tests/out/connection-l1/; the library and src/ are untouched)
##   prod: the production application and library as pinned (tests/l1/dependencies.lock.json): step 2b
##     (stage 6, INV_FOLLOW_CONNECT) = VAC copy as feedforward until the predicted contact, current PI from the
##     relay request, PLL frozen 15 ms from the relay command, synthesized feedforward from the predicted contact
##     to the end of the blanking, GRID_CONNECTED 20 ms after the command, output advance 0.3.
##   prod_kp2: the same with Kp 2 V/A, Ti 20 ms (a copy of src/app/control_step.h only).
## @par Runs (2 s each, 50 ns step; about 30 min on Jean's machine, 70 min on Luiz's)
##   step 2 (stage 3) @60 µs IEC; step 2b @80 µs IEC, cold bulb; @60 and @100 µs IEC; step 2b Kp 2 / Ti 20 ms @80 µs IEC.
"""Study builds only: the strategy becomes production through the library PR and src/app, with their tests."""
import argparse, json, os, re, shutil, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / 'tests/out/connection-l1'
LIB = REPO / 'owntech/lib/USB/single_phase_inverter'
SCEN = REPO / 'tests/l1/scenarios/inverter'

VARIANTS = {  # name: header constants to change (None: production as is)
    'prod': None,
    'prod_kp2': {'inverter_current_kp': 2.0, 'inverter_current_ti': 0.02}}
RUNS = [  # (variant, scenario, delay s, stage replacing CONNECT in the timeline)
    ('prod', 'bench_iec', 60e-6, 3), ('prod', 'bench_iec', 80e-6, 6), ('prod', 'bench_bulb', 80e-6, 6),
    ('prod', 'bench_iec', 60e-6, 6), ('prod', 'bench_iec', 100e-6, 6), ('prod_kp2', 'bench_iec', 80e-6, 6)]
# Closest averaged-model results (D-GFL-04/06: synthesized feedforward from the relay command; step 2b copies VAC
# until the contact instead, not simulated): peak line current before the step (A).
PREDICTED = {('prod', 'bench_iec', 60, 3): '9.8 (trips)', ('prod', 'bench_iec', 80, 6): '≈0.4 (D-GFL-04)',
             ('prod', 'bench_bulb', 80, 6): '≈3.8 (trips, D-GFL-06)', ('prod', 'bench_iec', 60, 6): '≈7 (trips, D-GFL-06)',
             ('prod', 'bench_iec', 100, 6): '≈7 (trips, D-GFL-06)', ('prod_kp2', 'bench_iec', 80, 6): '≈0.3 (D-GFL-06)'}


def rep(s, a, b):
    if s.count(a) != 1: raise SystemExit(f'patch anchor not found once: {a[:60]!r}')
    return s.replace(a, b)


def patch_app(dst, consts):
    (dst / 'app').mkdir(parents=True, exist_ok=True)
    hdr = (REPO / 'src/app/control_step.h').read_text()
    for const, val in consts.items():
        hdr, n = re.subn(rf'(inline constexpr float32_t {const} = )[^;]+;', rf'\g<1>{val!r}F;', hdr)
        if n != 1: raise SystemExit(f'constant not found once: {const}')
    (dst / 'app/control_step.h').write_text(hdr)


def check_library():
    """The study copies start from the local checkout: it must be the commit pinned for L1."""
    want = json.loads((REPO / 'tests/l1/dependencies.lock.json').read_text())['single_phase_inverter']['commit']
    have = subprocess.run(['git', '-C', str(LIB), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', '-C', str(LIB), 'status', '--porcelain', 'src'], capture_output=True, text=True).stdout.strip()
    if have != want or dirty:
        raise SystemExit(f'{LIB} is at {have[:7] or "?"}{" with local changes" if dirty else ""}; L1 pins {want[:7]} '
                         f'(git -C {LIB} fetch luizvilla && git -C {LIB} checkout {want[:7]})')


def build():
    check_library()
    for name, consts in VARIANTS.items():
        d = OUT / 'variants' / name; d.mkdir(parents=True, exist_ok=True)
        env = {**os.environ, 'SINGLE_PHASE_INVERTER_DIR': str(LIB), 'L1_BUILD_DIR': str(d / 'build')}
        if consts: patch_app(d / 'include', consts); env['L1_APP_INCLUDE'] = str(d / 'include')
        subprocess.run(['bash', str(REPO / 'tests/l1/firmware_block/build.sh')], check=True, env=env, stdout=subprocess.DEVNULL)
        print(name, '->', d / 'build/libcore_control.so')


def scenario(scen, delay, stage):
    s = json.loads((SCEN / f'{scen}.json').read_text())
    for q in s['commands']:
        if q.get('sync_request') == 3: q['sync_request'] = stage
    s['name'] += f'-stage{stage}'; s['title'] += f', stage {stage}'
    if abs(delay - 80e-6) > 1e-9:
        s['name'] += f'-delay{delay * 1e6:.0f}'; s['bridge'] = {**s.get('bridge', {}), 'update_delay_s': delay}
        s['title'] += f', actuation delay {delay * 1e6:.0f} µs'
    path = OUT / 'scenarios' / f'{scen}_stage{stage}_{delay * 1e6:.0f}us.json'; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(s, indent=2, ensure_ascii=False) + '\n'); return path


def run_name(v, scen, delay, stage): return f'{v}-stage{stage}-{scen}-{delay * 1e6:.0f}us'


def run(jobs):
    py = REPO / '.venv/bin/python'; py = py if py.exists() else Path(sys.executable)
    procs, queue = [], list(RUNS)
    while queue or procs:
        while queue and len(procs) < jobs:
            v, scen, delay, stage = queue.pop(0); name = run_name(v, scen, delay, stage); out = OUT / 'runs' / name; out.parent.mkdir(parents=True, exist_ok=True)
            env = {**os.environ, 'L1_CBLOCK_LIB': str(OUT / 'variants' / v / 'build/libcore_control.so')}
            log = open(OUT / 'runs' / f'{name}.log', 'w')
            procs.append((name, subprocess.Popen([str(py), 'runner.py', '--scenario', str(scenario(scen, delay, stage)), '--out', str(out)],
                                                 cwd=REPO / 'tests/l1', env=env, stdout=log, stderr=subprocess.STDOUT)))
            print('started', name)
        for item in list(procs):
            if item[1].poll() is not None: print('finished', item[0], 'exit', item[1].returncode); procs.remove(item)
        if procs: import time; time.sleep(20)


def report():
    sys.path.insert(0, str(Path(__file__).parent)); import analyze
    out = REPO / 'docs/testing/design/L1-GFL-14_IEC'; out.mkdir(parents=True, exist_ok=True)
    lines = ['# L1-GFL-14: connection on the switched model, step 2b (VAC copy, PI from the relay request, blanking)', '',
             'Switched model, real application through the bench stages (contract app-v3), production builds of `tests/l1/design/connection_l1.py`. '
             'Step 2b (stage 6): VAC copy as feedforward until the predicted contact, current PI from the relay request, PLL frozen 15 ms from the relay command, '
             'synthesized feedforward from the predicted contact to the end of the blanking, GRID_CONNECTED 20 ms after the command, output advance 0.3. '
             'Step 2 (stage 3) is today\'s connection. The relay contact falls on a voltage zero crossing (the application\'s latch).', '',
             '| Run | Closest averaged result: peak (A) | Fault | Contact from zero crossing (°) | Closing peak, 100 ms (A) | Residual (A RMS) | Final I_d (A) | THD (%) |', '|---|---|---|---|---|---|---|---|']
    for v, scen, delay, stage in RUNS:
        d = OUT / 'runs' / run_name(v, scen, delay, stage); key = (v, scen, round(delay * 1e6), stage)
        if not (d / 'control.csv').exists(): lines.append(f'| {run_name(v, scen, delay, stage)} | {PREDICTED[key]} | not run | | | | | |'); continue
        r = analyze.analyze(d); fz = r.get('fault'); fmt = lambda x, s=1: '—' if x is None or x != x else f'{x * s:.3g}'
        fault = f"{fz[0]:.4f} s, code {fz[1][4]}{', bench trip' if fz[1][5] else ''}" if fz else 'none'
        lines.append(f"| {run_name(v, scen, delay, stage)} | {PREDICTED[key]} | {fault} | {fmt(r.get('contact_zc_deg'))} | {fmt(r.get('closing_peak_A'))} | "
                     f"{fmt(r.get('residual_rms_A'))} | {fmt(r.get('final_id_A'))} | {fmt(r.get('thd'), 100)} |")
    (out / 'report.md').write_text('\n'.join(lines) + '\n'); print('\n'.join(lines))


def main():
    a = argparse.ArgumentParser(); a.add_argument('cmd', choices=['build', 'run', 'report']); a.add_argument('--jobs', type=int, default=len(RUNS))
    o = a.parse_args(); {'build': build, 'run': lambda: run(o.jobs), 'report': report}[o.cmd]()


if __name__ == '__main__':
    main()
