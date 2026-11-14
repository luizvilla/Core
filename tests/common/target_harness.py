#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file target_harness.py
## @brief Shared helpers of the on-target safety tests (bare Spin + ST-Link): build and flash a test env, find a
##        symbol, read and write target RAM over SWD without halting, drive a command/acknowledge protocol that
##        rides through resets, print result lines. Standard library only.
import json
import os
import struct
import subprocess
import sys
import tempfile
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PIO = os.path.expanduser(os.environ.get('PIO', '~/.platformio/penv/bin/pio'))
PKG = os.path.expanduser(os.environ.get('PLATFORMIO_PACKAGES_DIR', '~/.platformio/packages'))
OPENOCD = os.path.join(PKG, 'tool-openocd', 'bin', 'openocd')
OPENOCD_SCRIPTS = os.path.join(PKG, 'tool-openocd', 'openocd', 'scripts')
NM = os.path.join(PKG, 'toolchain-gccarmnoneeabi', 'bin', 'arm-none-eabi-nm')


def elf(env):
    return os.path.join(REPO, '.pio', 'build', env, 'firmware.elf')


def flash(env, log_path):
    """@brief Builds and flashes env over the ST-Link; restores zephyr/CMakeLists.txt's board lines. @return rc."""
    with open(log_path, 'w') as f:
        r = subprocess.run([PIO, 'run', '-e', env, '-t', 'upload'], cwd=REPO, stdout=f, stderr=subprocess.STDOUT)
    subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'restore_board_lines.py')], cwd=REPO)
    return r.returncode


def symbol(env, name):
    out = subprocess.run([NM, '-S', elf(env)], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[3] == name:
            return int(p[0], 16), int(p[1], 16)
    sys.exit(f'target_harness: symbol {name} not in {elf(env)}')


def symbol_at(env, addr):
    """@brief Name+offset of the data symbol containing addr, '-' for 0."""
    if not addr:
        return '-'
    out = subprocess.run([NM, '-S', elf(env)], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and int(p[0], 16) <= addr < int(p[0], 16) + int(p[1], 16):
            return f'{p[3]}+{addr - int(p[0], 16)}'
    return '?'


def openocd(*cmds, check=True):
    args = [OPENOCD, '-d0', '-s', OPENOCD_SCRIPTS, '-f', 'interface/stlink.cfg', '-f', 'target/stm32g4x.cfg',
            '-c', 'init']
    for c in cmds:
        args += ['-c', c]
    args += ['-c', 'shutdown']
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if check and r.returncode != 0:
        raise RuntimeError(r.stderr[-1500:])
    return r


def read_words(addr, n):
    """@brief n 32-bit words of target memory, read without halting; None on an SWD error (e.g. during a reset)."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'm.bin')
        r = openocd(f'dump_image {path} {addr:#x} {4 * n}', check=False)
        if r.returncode != 0 or not os.path.exists(path) or os.path.getsize(path) != 4 * n:
            return None
        return list(struct.unpack(f'<{n}I', open(path, 'rb').read()))


def write_word(addr, value):
    openocd(f'mww {addr:#x} {value:#x}')


def idcode():
    out = openocd('mdw 0xE0042000', check=False)
    text = out.stdout + out.stderr
    return next((int(l.split()[1], 16) for l in text.splitlines() if l.startswith('0xe0042000:')), None)


class Protocol:
    """Command/acknowledge over SWD: the firmware publishes a report struct (magic, layout, boot_count, ack, busy,
    then its own fields) and polls a command word (sequence << 8) | case."""

    def __init__(self, env, report_symbol, cmd_symbol, fields, magic):
        self.env = env
        self.addr, size = symbol(env, report_symbol)
        self.cmd, _ = symbol(env, cmd_symbol)
        self.fields = fields
        self.magic = magic
        if size != 4 * len(fields):
            sys.exit(f'target_harness: {report_symbol} is {size} bytes, the reader expects {4 * len(fields)}')

    def read(self):
        w = read_words(self.addr, len(self.fields))
        if w is None or w[0] != self.magic:
            return None
        return dict(zip(self.fields, w))

    def wait(self, pred, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rep = self.read()
            if rep is not None and pred(rep):
                return rep
            time.sleep(0.5)
        raise TimeoutError(what)

    def run(self, name, code, timeout=20):
        """@brief Runs one case; @return (report after, entry for the log)."""
        before = self.wait(lambda r: r['busy'] == 0, 30, 'firmware not idle')
        seq = (before['ack'] & 0xFFFFFF) + 1
        t0 = time.monotonic()
        write_word(self.cmd, (seq << 8) | code)
        try:
            rep = self.wait(lambda r: r['ack'] == seq and r['busy'] == 0, timeout, f'{name} not acknowledged')
            status = 'acknowledged'
        except TimeoutError as e:
            rep = self.read() or before
            status = str(e)
        entry = {'case': name, 'seq': seq, 'status': status, 'seconds': round(time.monotonic() - t0, 2),
                 'boots': rep['boot_count'] - before['boot_count']}
        print(f"== {name}: {status}, {entry['boots']} reset(s), {entry['seconds']} s", flush=True)
        return rep, entry


def line(cid, verdict, text, expected, measured):
    print(f'@@\t{cid}\t{verdict}\t{text}\t{expected}\t{measured}', flush=True)


def judge(ok):
    return 'PASS' if ok else 'FAIL'


def git_state():
    sha = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=REPO, capture_output=True,
                           text=True).stdout.strip()
    return sha + ('-dirty' if dirty else '')


def write_target_manifest(out, title, purpose, criteria, conditions, env, figures, report='report.json'):
    idc = idcode()
    target = [['Board', 'Spin, bare (no shield, no power stage)'],
              ['MCU', f'STM32G474RE, DBGMCU_IDCODE {idc:#010x}' if idc is not None else 'STM32G474RE'],
              ['Firmware', f'env {env} at {git_state()}'],
              ['Probe', 'ST-Link over SWD, PlatformIO OpenOCD; results read from RAM and TAMP backup registers']]
    with open(os.path.join(out, 'target_manifest.json'), 'w') as f:
        json.dump({'title': title, 'purpose': purpose, 'criteria': criteria, 'conditions': conditions,
                   'target': target, 'report': report, 'figures': figures}, f, indent=1)
