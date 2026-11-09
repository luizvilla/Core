#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file variant.py
## @brief Build a study C-Block from a patched copy of single_phase_inverter (current-loop candidates).
## @par Usage
##   python3 tests/l1/design/variant.py NAME --kr-i 2000 --kp 1.0 --ti 0.005 --comp 0.5 --out DIR
## @par What it patches (copy only; the pinned library is untouched)
##   the current PI gains, the current SOGI gain (separate from the voltage SOGI), and an advance of
##   the output rotation by comp control periods (delay compensation). Prints the library path.
"""Study builds only: a candidate becomes production only through a library change and its tests."""
import argparse, os, shutil, subprocess
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]
SRC = REPO / 'owntech/lib/USB/single_phase_inverter'

def main():
    a = argparse.ArgumentParser(); a.add_argument('name'); a.add_argument('--kr-i', type=float, default=500.0)
    a.add_argument('--kp', type=float, default=0.001); a.add_argument('--ti', type=float, default=0.0003)
    a.add_argument('--comp', type=float, default=0.0); a.add_argument('--out', type=Path, required=True)
    o = a.parse_args(); lib = o.out / o.name / 'single_phase_inverter'; build = o.out / o.name / 'build'
    if lib.exists(): shutil.rmtree(lib)
    shutil.copytree(SRC, lib, ignore=shutil.ignore_patterns('.git'))
    f = lib / 'src/singlePhaseInverterFollowing.cpp'; s = f.read_text()
    def rep(x, y):
        nonlocal s
        if s.count(x) != 1: raise SystemExit(f'patch anchor not unique: {x}')
        s = s.replace(x, y)
    rep('_current_pi_params.Kp = 0.001;', f'_current_pi_params.Kp = {o.kp!r}F;')
    rep('_current_pi_params.Ti = 0.0003;', f'_current_pi_params.Ti = {o.ti!r}F;')
    rep('_sogi_i.init(500.0F, _Ts);', f'_sogi_i.init({o.kr_i!r}F, _Ts);')  # fault reset first: the next edit
    rep('_sogi_i.init(Kr, _Ts);', f'_sogi_i.init({o.kr_i!r}F, _Ts);')       # can create the same text
    rep('_Vab_output = Transform::rotation_to_clarke(_Vdq_output, _theta);',
        f'_Vab_output = Transform::rotation_to_clarke(_Vdq_output, _theta + {o.comp!r}F * _w * _Ts);')
    f.write_text(s); build.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, 'SINGLE_PHASE_INVERTER_DIR': str(lib), 'L1_BUILD_DIR': str(build)}
    subprocess.run(['bash', str(REPO / 'tests/l1/firmware_block/build.sh')], check=True, env=env, stdout=subprocess.DEVNULL)
    print(build / 'libcore_control.so')

if __name__ == '__main__':
    main()
