#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file console_reader.py
## @brief Reads the Spin's USB console during an on-target timing run, as a bench terminal does (L0-TIM-03).
##
## Opens the Spin's CDC ACM port (USB id 2fe3:0100, /dev/serial/by-id/*SPIN* or *2fe3*), retrying while the board
## boots, reads everything into a log, and sends the menu key 'h' every 5 s and the scope download key 'r' every
## 10 s, as a person at the bench terminal does; each key sent is marked in the log ([console_reader: sent x]).
## Every line is prefixed with the host time in seconds since the reader started ("  12.345 | ").
## Stops after --seconds or when the file named by --stop-file appears. Standard library only (Linux).
##
## @par Usage
##   python3 tests/l0/timing/console_reader.py --out console.log --seconds 240 [--stop-file F]
import argparse
import glob
import os
import sys
import termios
import time
import tty


def find_port():
    for pattern in ('/dev/serial/by-id/*SPIN*', '/dev/serial/by-id/*2fe3*', '/dev/serial/by-id/*OwnTech*'):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=240.0)
    ap.add_argument('--stop-file')
    a = ap.parse_args()
    t0 = time.monotonic()
    end = t0 + a.seconds
    received = 0
    pending = b''
    with open(a.out, 'wb') as raw:
        class Log:
            """Writes complete lines with the host time in front."""
            def write(self, data):
                nonlocal pending
                pending += data
                *lines, pending = pending.split(b'\n')
                for ln in lines:
                    raw.write(f'{time.monotonic() - t0:9.3f} | '.encode() + ln.rstrip(b'\r') + b'\n')
        log = Log()
        fd = None
        last_key = 0.0
        last_r = time.monotonic()
        while time.monotonic() < end and not (a.stop_file and os.path.exists(a.stop_file)):
            if fd is None:
                port = find_port()
                if port is None:
                    time.sleep(0.5)
                    continue
                try:
                    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    tty.setraw(fd)
                    attrs = termios.tcgetattr(fd)
                    attrs[4] = attrs[5] = termios.B115200
                    termios.tcsetattr(fd, termios.TCSANOW, attrs)
                    log.write(f'\n[console_reader: opened {port}]\n'.encode())
                except OSError:
                    fd = None
                    time.sleep(0.5)
                    continue
            try:
                chunk = os.read(fd, 4096)
                if chunk:
                    received += len(chunk)
                    log.write(chunk)
                else:
                    time.sleep(0.01)
                if time.monotonic() - last_key > 5.0:
                    os.write(fd, b'h')
                    log.write(b'\n[console_reader: sent h]\n')
                    last_key = time.monotonic()
                if time.monotonic() - last_r > 10.0:
                    os.write(fd, b'r')
                    log.write(b'\n[console_reader: sent r]\n')
                    last_r = time.monotonic()
            except BlockingIOError:
                time.sleep(0.01)
            except OSError:  # board reset or unplugged: reopen
                os.close(fd)
                fd = None
                log.write(b'\n[console_reader: port lost, reopening]\n')
        if fd is not None:
            os.close(fd)
        if pending:
            log.write(b'\n')
    print(f'console_reader: {received} bytes', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
