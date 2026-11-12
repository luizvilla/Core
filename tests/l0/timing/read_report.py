#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file read_report.py
## @brief Reads g_timing_report from the running Spin over SWD (ST-Link + PlatformIO's OpenOCD).
##
## Finds the symbol in the ELF with arm-none-eabi-nm, waits until the harness reports DONE (or ERROR), dumps the
## structure with OpenOCD and writes it as JSON. Standard library only. Mirrors src/timing/timing_report.h
## (layout version 5).
##
## @par Usage
##   python3 tests/l0/timing/read_report.py --elf .pio/build/TIMING/firmware.elf --out report.json [--timeout 120]
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
import time

LAYOUT_VERSION = 5
MAGIC = 0x314D4954
MAX_BUCKETS = 16
MAX_ROWS = 640
HEADER = struct.Struct('<8I')
BUCKET = struct.Struct('<16sIIBB2xIIIII')
ROW = struct.Struct('<32sIIIIQ')
SIZE = HEADER.size + BUCKET.size * MAX_BUCKETS + ROW.size * MAX_ROWS
STATES = {0: 'RUNNING', 1: 'DONE', 2: 'ERROR', 3: 'RUNNING_TIM02', 4: 'RUNNING_TIM03'}
PIO_PACKAGES = os.path.expanduser(os.environ.get('PLATFORMIO_PACKAGES_DIR', '~/.platformio/packages'))
OPENOCD = os.path.join(PIO_PACKAGES, 'tool-openocd', 'bin', 'openocd')
OPENOCD_SCRIPTS = os.path.join(PIO_PACKAGES, 'tool-openocd', 'openocd', 'scripts')
NM = os.path.join(PIO_PACKAGES, 'toolchain-gccarmnoneeabi', 'bin', 'arm-none-eabi-nm')


def symbol(elf, name):
    """@brief Address and size of a data symbol. @return (address, size)."""
    out = subprocess.run([NM, '-S', elf], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] == name:
            return int(parts[0], 16), int(parts[1], 16)
    sys.exit(f'read_report: symbol {name} not in {elf} (not a TIMING build?)')


def openocd(*commands):
    """@brief Runs OpenOCD on the ST-Link with the given commands, without halting or resetting the core."""
    args = [OPENOCD, '-d0', '-s', OPENOCD_SCRIPTS, '-f', 'interface/stlink.cfg', '-f', 'target/stm32g4x.cfg',
            '-c', 'init']
    for c in commands:
        args += ['-c', c]
    args += ['-c', 'shutdown']
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        sys.exit(f'read_report: OpenOCD failed (ST-Link connected? udev rules?):\n{r.stderr[-2000:]}')
    return r.stdout + r.stderr


def dump(address, size):
    """@brief Raw bytes of target memory."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'report.bin')
        openocd(f'dump_image {path} {address:#x} {size}')
        with open(path, 'rb') as f:
            return f.read()


def target_registers():
    """@brief Device id/revision (DBGMCU_IDCODE) and flash configuration (FLASH_ACR) of the running target."""
    out = openocd('mdw 0xE0042000', 'mdw 0x40022000')
    vals = {}
    for ln in out.splitlines():
        if ln.startswith('0xe0042000:'):
            vals['dbgmcu_idcode'] = int(ln.split()[1], 16)
        elif ln.startswith('0x40022000:'):
            vals['flash_acr'] = int(ln.split()[1], 16)
    acr = vals.get('flash_acr')
    if acr is not None:
        vals['flash_wait_states'] = acr & 0xF
        vals['flash_prefetch'] = bool(acr >> 8 & 1)
        vals['flash_icache'] = bool(acr >> 9 & 1)
        vals['flash_dcache'] = bool(acr >> 10 & 1)
    idc = vals.get('dbgmcu_idcode')
    if idc is not None:
        vals['device_id'] = idc & 0xFFF
        vals['revision_id'] = idc >> 16
    return vals


def parse(raw):
    """@brief Decodes the report. @return dict."""
    magic, version, state, core_hz, nb, nr, err, ticks = HEADER.unpack_from(raw, 0)
    if magic != MAGIC:
        return {'state': 'NOT_STARTED', 'magic': magic}
    if version != LAYOUT_VERSION:
        sys.exit(f'read_report: layout version {version} on target, reader knows {LAYOUT_VERSION}')
    rep = {'state': STATES.get(state, str(state)), 'core_hz': core_hz, 'error_code': err, 'total_ticks': ticks,
           'buckets': [], 'rows': []}
    for i in range(min(nb, MAX_BUCKETS)):
        name, t, mask, final, mode, t2, mask2, t3, mask3, lost3 = BUCKET.unpack_from(raw, HEADER.size + i * BUCKET.size)
        rep['buckets'].append({'name': name.split(b'\0')[0].decode(), 'ticks': t,
                               'vsi_seen': [m for m in range(32) if mask >> m & 1],
                               'final_vsi': final, 'boost_mode': mode, 'tim02_ticks': t2,
                               'tim02_vsi_seen': [m for m in range(32) if mask2 >> m & 1], 'tim03_ticks': t3,
                               'tim03_vsi_seen': [m for m in range(32) if mask3 >> m & 1], 'tim03_lost': lost3})
    base = HEADER.size + BUCKET.size * MAX_BUCKETS
    for i in range(min(nr, MAX_ROWS)):
        name, calls, mn, mx, mx_tick, total = ROW.unpack_from(raw, base + i * ROW.size)
        rep['rows'].append({'name': name.split(b'\0')[0].decode(), 'calls': calls, 'min_cycles': mn,
                            'max_cycles': mx, 'max_tick': mx_tick, 'sum_cycles': total})
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--elf', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=120.0, help='seconds to wait for DONE')
    a = ap.parse_args()
    address, size = symbol(a.elf, 'g_timing_report')
    if size != SIZE:
        sys.exit(f'read_report: g_timing_report is {size} bytes, reader expects {SIZE} (layout mismatch)')
    deadline = time.monotonic() + a.timeout
    while True:
        rep = parse(dump(address, SIZE))
        print(f"read_report: {rep['state']}, {rep.get('total_ticks', 0)} ticks", file=sys.stderr)
        if rep['state'] in ('DONE', 'ERROR') or time.monotonic() > deadline:
            break
        time.sleep(3.0)
    rep['target'] = target_registers()
    with open(a.out, 'w') as f:
        json.dump(rep, f, indent=1)
    return 0 if rep['state'] == 'DONE' else 1


if __name__ == '__main__':
    sys.exit(main())
