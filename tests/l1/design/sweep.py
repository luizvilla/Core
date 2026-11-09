#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file sweep.py
## @brief Build study C-Blocks that differ only in the application's current-loop constants.
## @par Usage
##   python3 tests/l1/design/sweep.py --lib LIB_DIR --out DIR NAME:KP:TI [NAME:KP:TI ...]
## @par What it changes (copies only)
##   app::inverter_current_kp and app::inverter_current_ti in a copy of src/app/control_step.h,
##   found before src/ through L1_APP_INCLUDE; the library is LIB_DIR (SINGLE_PHASE_INVERTER_DIR).
##   Prints one libcore_control.so path per variant.
"""Study builds only: tuning becomes production only through src/app/control_step.h and its tests."""
import argparse, os, re, subprocess
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]


def main():
    a = argparse.ArgumentParser(); a.add_argument('--lib', type=Path, required=True)
    a.add_argument('--out', type=Path, required=True); a.add_argument('variants', nargs='+')
    o = a.parse_args(); header = (REPO / 'src/app/control_step.h').read_text()
    for v in o.variants:
        name, kp, ti = v.split(':'); s = header
        for const, val in (('inverter_current_kp', kp), ('inverter_current_ti', ti)):
            s, n = re.subn(rf'(inline constexpr float32_t {const} = )[^;]+;', rf'\g<1>{float(val)!r}F;', s)
            if n != 1: raise SystemExit(f'constant not found once: {const}')
        inc = o.out / name / 'include'; (inc / 'app').mkdir(parents=True, exist_ok=True)
        (inc / 'app/control_step.h').write_text(s); build = o.out / name / 'build'; build.mkdir(parents=True, exist_ok=True)
        env = {**os.environ, 'SINGLE_PHASE_INVERTER_DIR': str(o.lib.resolve()), 'L1_BUILD_DIR': str(build.resolve()),
               'L1_APP_INCLUDE': str(inc.resolve())}
        subprocess.run(['bash', str(REPO / 'tests/l1/firmware_block/build.sh')], check=True, env=env, stdout=subprocess.DEVNULL)
        print(build / 'libcore_control.so')


if __name__ == '__main__':
    main()
