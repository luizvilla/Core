#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file connection_l1.py
## @brief Switched-model validation (L1-GFL-14) of the connection strategy found by the design tests D-GFL-04/06.
## @par Usage (from the repository root)
##   python3 tests/l1/design/connection_l1.py build            # study builds (a minute)
##   python3 tests/l1/design/connection_l1.py run [--jobs N]   # the six runs, N in parallel (default: all)
##   python3 tests/l1/design/connection_l1.py report           # docs/testing/design/L1-GFL-14_IEC/report.md
## @par Study builds (copies in tests/out/connection-l1/; the library and src/ are untouched)
##   library: luizvilla/single_phase_inverter as pinned in tests/l1/dependencies.lock.json, plus
##     - the current PI also active in GRID_CONNECT (zero reference), as in the design tests;
##     - holdConnectionReference(duration): for that long, the feedforward is the period-averaged Vd (Vq 0)
##       rotated by the PLL angle and the PLL frequency is frozen at its period average.
##   application: a copy of src/app/control_step.{h,cpp}; when the relay command is latched it calls
##     holdConnectionReference(relay_close_delay + 5 ms), i.e. until 5 ms after the contact; output advance 0.3.
## @par Runs (2 s each, 50 ns step; about 30 min on Jean's machine, 70 min on Luiz's)
##   today@60µs IEC, best@80µs IEC, best@80µs cold bulb, best@60µs IEC, best@100µs IEC, best Kp 2 / Ti 20 ms @80µs IEC.
"""Study builds only: the strategy becomes production through the library PR and src/app, with their tests."""
import argparse, json, os, re, shutil, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / 'tests/out/connection-l1'
LIB = REPO / 'owntech/lib/USB/single_phase_inverter'
SCEN = REPO / 'tests/l1/scenarios/inverter'
HOLD_AFTER_CONTACT = 0.005

VARIANTS = {  # name: (hold, advance, kp, ti)
    'today': (False, 0.5, 1.0, 0.005),
    'best': (True, 0.3, 1.0, 0.005),
    'best_kp2': (True, 0.3, 2.0, 0.02)}
RUNS = [  # (variant, scenario, delay s)
    ('today', 'bench_iec', 60e-6), ('best', 'bench_iec', 80e-6), ('best', 'bench_bulb', 80e-6),
    ('best', 'bench_iec', 60e-6), ('best', 'bench_iec', 100e-6), ('best_kp2', 'bench_iec', 80e-6)]
# Averaged-model predictions (D-GFL-06, contact at 0°/45°): peak line current before the step (A).
PREDICTED = {('today', 'bench_iec', 60): '9.8 (trips)', ('best', 'bench_iec', 80): '0.37–0.41',
             ('best', 'bench_bulb', 80): '3.8 (trips)', ('best', 'bench_iec', 60): '7.0 (trips)',
             ('best', 'bench_iec', 100): '7.3 (trips)', ('best_kp2', 'bench_iec', 80): '0.25–0.32'}


def rep(s, a, b):
    if s.count(a) != 1: raise SystemExit(f'patch anchor not found once: {a[:60]!r}')
    return s.replace(a, b)


def patch_library(dst, hold):
    if dst.exists(): shutil.rmtree(dst)
    shutil.copytree(LIB, dst, ignore=shutil.ignore_patterns('.git'))
    c = dst / 'src/singlePhaseInverterFollowing.cpp'; h = dst / 'src/singlePhaseInverterFollowing.h'; s = c.read_text()
    s = rep(s, """    if (_mode == VSI_RUN || _mode == VSI_GRID_CONNECTED ||
        _mode == VSI_STOPPING) {""", """    if (_mode == VSI_RUN || _mode == VSI_GRID_CONNECTED ||
        _mode == VSI_STOPPING || _mode == VSI_GRID_CONNECT) {  // STUDY: PI from the relay request""")
    if hold:
        s = rep(s, "    _w = _w_ref + _pll_pi.calculateWithReturn(0, -1.0F * _Vdq.q);",
                "    if (_hold_remaining > 0.0F) { _w = _hold_w; _hold_remaining -= _Ts; }  // STUDY: PLL frozen during the hold\n"
                "    else { _w = _w_ref + _pll_pi.calculateWithReturn(0, -1.0F * _Vdq.q); }")
        s = rep(s, "        _Vdq_output.d = _Vdq.d + _current_d_pi.calculateWithReturn(id_ref, _Idq.d);\n"
                   "        _Vdq_output.q = _Vdq.q + _current_q_pi.calculateWithReturn(iq_ref, _Idq.q);",
                "        const bool hold_ff = _hold_remaining > 0.0F;  // STUDY: synthesized feedforward during the hold\n"
                "        _Vdq_output.d = (hold_ff ? _hold_vd : _Vdq.d) + _current_d_pi.calculateWithReturn(id_ref, _Idq.d);\n"
                "        _Vdq_output.q = (hold_ff ? 0.0F : _Vdq.q) + _current_q_pi.calculateWithReturn(iq_ref, _Idq.q);")
        s += """
// STUDY (tests/l1/design/connection_l1.py): connection reference hold.
void singlePhaseInverterFollowing::holdConnectionReference(float32_t duration) {
    _hold_vd = isSyncWindowFull() ? _Vdq_avg.d : _Vdq.d;
    _hold_w = isSyncWindowFull() ? _w_avg : _w;
    _hold_remaining = duration;
}
"""
        t = h.read_text()
        t = rep(t, "    /** @brief Read the output-rotation advance in control periods. */",
                "    /** @brief STUDY: synthesized feedforward and frozen PLL for duration (s). */\n"
                "    void holdConnectionReference(float32_t duration);\n\n"
                "    /** @brief Read the output-rotation advance in control periods. */")
        t = rep(t, "    float32_t _output_phase_advance = 0.0F;",
                "    float32_t _output_phase_advance = 0.0F;\n"
                "    float32_t _hold_remaining = 0.0F, _hold_vd = 0.0F, _hold_w = 0.0F;  ///< STUDY: connection hold.")
        h.write_text(t)
    c.write_text(s)


def patch_app(dst, hold, adv, kp, ti):
    (dst / 'app').mkdir(parents=True, exist_ok=True)
    hdr = (REPO / 'src/app/control_step.h').read_text()
    for const, val in (('inverter_output_phase_advance', adv), ('inverter_current_kp', kp), ('inverter_current_ti', ti)):
        hdr, n = re.subn(rf'(inline constexpr float32_t {const} = )[^;]+;', rf'\g<1>{val!r}F;', hdr)
        if n != 1: raise SystemExit(f'constant not found once: {const}')
    (dst / 'app/control_step.h').write_text(hdr)
    src = (REPO / 'src/app/control_step.cpp').read_text()
    if hold:
        call = f'{{ state.relay_command_latched = true; state.inverter.holdConnectionReference(app::relay_close_delay + {HOLD_AFTER_CONTACT!r}F); }}'
        src = rep(src, "            if (phi < 1.5F * w * app::hot_loop_period) state.relay_command_latched = true;",
                  f"            if (phi < 1.5F * w * app::hot_loop_period) {call}  // STUDY: hold until 5 ms after the contact")
    (dst / 'control_step.cpp').write_text(src)


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
    for name, (hold, adv, kp, ti) in VARIANTS.items():
        d = OUT / 'variants' / name; d.mkdir(parents=True, exist_ok=True)
        patch_library(d / 'single_phase_inverter', hold); patch_app(d / 'include', hold, adv, kp, ti)
        env = {**os.environ, 'SINGLE_PHASE_INVERTER_DIR': str(d / 'single_phase_inverter'), 'L1_BUILD_DIR': str(d / 'build'),
               'L1_APP_INCLUDE': str(d / 'include'), 'L1_APP_SRC': str(d / 'include/control_step.cpp')}
        subprocess.run(['bash', str(REPO / 'tests/l1/firmware_block/build.sh')], check=True, env=env, stdout=subprocess.DEVNULL)
        print(name, '->', d / 'build/libcore_control.so')


def scenario(scen, delay):
    s = json.loads((SCEN / f'{scen}.json').read_text())
    if abs(delay - 80e-6) > 1e-9:
        s['name'] += f'-delay{delay * 1e6:.0f}'; s['bridge'] = {**s.get('bridge', {}), 'update_delay_s': delay}
        s['title'] += f', actuation delay {delay * 1e6:.0f} µs'
    path = OUT / 'scenarios' / f'{scen}_{delay * 1e6:.0f}us.json'; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(s, indent=2, ensure_ascii=False) + '\n'); return path


def run_name(v, scen, delay): return f'{v}-{scen}-{delay * 1e6:.0f}us'


def run(jobs):
    py = REPO / '.venv/bin/python'; py = py if py.exists() else Path(sys.executable)
    procs, queue = [], list(RUNS)
    while queue or procs:
        while queue and len(procs) < jobs:
            v, scen, delay = queue.pop(0); name = run_name(v, scen, delay); out = OUT / 'runs' / name; out.parent.mkdir(parents=True, exist_ok=True)
            env = {**os.environ, 'L1_CBLOCK_LIB': str(OUT / 'variants' / v / 'build/libcore_control.so')}
            log = open(OUT / 'runs' / f'{name}.log', 'w')
            procs.append((name, subprocess.Popen([str(py), 'runner.py', '--scenario', str(scenario(scen, delay)), '--out', str(out)],
                                                 cwd=REPO / 'tests/l1', env=env, stdout=log, stderr=subprocess.STDOUT)))
            print('started', name)
        for item in list(procs):
            if item[1].poll() is not None: print('finished', item[0], 'exit', item[1].returncode); procs.remove(item)
        if procs: import time; time.sleep(20)


def report():
    sys.path.insert(0, str(Path(__file__).parent)); import analyze
    out = REPO / 'docs/testing/design/L1-GFL-14_IEC'; out.mkdir(parents=True, exist_ok=True)
    lines = ['# L1-GFL-14: connection strategy of D-GFL-04 on the switched model', '',
             'Switched model, real application through the bench stages (contract app-v3), study builds of `tests/l1/design/connection_l1.py`. '
             'Strategy: current PI from the relay request, output advance 0.3 period, synthesized feedforward and PLL frozen from the relay command until 5 ms after the contact. '
             '"today" is the same build without the hold, advance 0.5. The relay contact falls on a voltage zero crossing (the application\'s latch).', '',
             '| Run | Averaged prediction: peak (A) | Fault | Contact from zero crossing (°) | Closing peak, 100 ms (A) | Residual (A RMS) | Final I_d (A) | THD (%) |', '|---|---|---|---|---|---|---|---|']
    for v, scen, delay in RUNS:
        d = OUT / 'runs' / run_name(v, scen, delay)
        if not (d / 'control.csv').exists(): lines.append(f'| {run_name(v, scen, delay)} | {PREDICTED[(v, scen, round(delay * 1e6))]} | not run | | | | | |'); continue
        r = analyze.analyze(d); fz = r.get('fault'); fmt = lambda x, s=1: '—' if x is None or x != x else f'{x * s:.3g}'
        fault = f"{fz[0]:.4f} s, code {fz[1][4]}{', bench trip' if fz[1][5] else ''}" if fz else 'none'
        lines.append(f"| {run_name(v, scen, delay)} | {PREDICTED[(v, scen, round(delay * 1e6))]} | {fault} | {fmt(r.get('contact_zc_deg'))} | {fmt(r.get('closing_peak_A'))} | "
                     f"{fmt(r.get('residual_rms_A'))} | {fmt(r.get('final_id_A'))} | {fmt(r.get('thd'), 100)} |")
    (out / 'report.md').write_text('\n'.join(lines) + '\n'); print('\n'.join(lines))


def main():
    a = argparse.ArgumentParser(); a.add_argument('cmd', choices=['build', 'run', 'report']); a.add_argument('--jobs', type=int, default=len(RUNS))
    o = a.parse_args(); {'build': build, 'run': lambda: run(o.jobs), 'report': report}[o.cmd]()


if __name__ == '__main__':
    main()
