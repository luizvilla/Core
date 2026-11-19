#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file trim.py
## @brief SPOC bench: compensation of the four scope channels' dividers, for trimming (L0-CAL, issue luizvilla/Core#26).
##
## The CUT drives PA8 at 1 kHz, 0.5; the MUX routes PA8 to one channel at a time (MuxController.route() disables
## every other route first); each channel's compensation is the level
## 5-25 us after each edge relative to the settled level (Core4 analysis.compensation_pct): 100 % = trimmed,
## above = over-compensated (spike that decays, turn the trimmer down), below = rounded corner (turn it up).
## Trimmed: 95-105 %. Needs the CUT running Core-CUT and the MUX running Core-MUX (mux-injector), scope on USB.
##
## @par Usage
##   .venv/bin/python tests/l0/spoc_cal/trim.py                  # one reading per channel, each routed alone
##   .venv/bin/python tests/l0/spoc_cal/trim.py --live --ch 3    # CH3 only, refreshed until Ctrl-C, while trimming
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
COMMON = os.path.join(HERE, '..', '..', 'common')
sys.path[:0] = [COMMON, os.path.join(COMMON, 'spoc')]
import rigol_mho  # noqa: E402
from spoc import boards, waveform  # noqa: E402
from mux_controller import MuxController  # noqa: E402
from thingset import ThingSetShell  # noqa: E402,F401

CUT_PIN = 'PA8'
CHANNELS = (1, 2, 3, 4)
TRIMMED = (95.0, 105.0)


def setup(scope, mux, cut):
    cut_pwm_on(cut)
    mux.disable_all()
    for ch in CHANNELS:
        scope.setup_channel(ch, scale=1.0, offset=-1.65, probe=10)
    scope.set_acquire('NORMal')
    scope.set_timebase(200e-6)
    scope.set_edge_trigger(1, 1.65)
    scope.run()


def cut_pwm_on(port):
    """1 kHz, 0.5 on PA8 through Core-CUT's /Cut/PwmOut (unit A, output 1)."""
    import serial
    ser = serial.Serial(port, 115200, timeout=0.2)
    sh = ThingSetShell(ser)
    sh.enter_thingset()
    for path, value in (('wFreq_Hz', '1000'), ('wMinFreq_Hz', '1000'), ('wA1', 'true'), ('wADuty', '0.5'),
                        ('wEnable', 'true'), ('xApply', 'true')):
        ok, resp = sh.set_value(f'Cut/PwmOut/{path}', value)
        if not ok:
            raise RuntimeError(f'CUT write failed for {path}: {resp}')
    ser.close()


def route_alone(mux, ch):
    mux.disable_all()
    mux.route(f'ch{ch}', CUT_PIN)
    time.sleep(1.2)


def read(scope, channels, mux=None):
    """Compensation of each channel; with mux given, each channel is routed alone first."""
    out = {}
    for ch in channels:
        if mux is not None:
            route_alone(mux, ch)
        t, v = scope.waveform(ch)
        base, top = waveform.levels(v)
        out[ch] = {'comp': waveform.compensation_pct(t, v), 'base': base, 'top': top}
    return out


def line(values):
    parts = []
    for ch, r in values.items():
        c = r['comp']
        if r['top'] - r['base'] < 1.0:
            parts.append(f'CH{ch}  no signal  ')
            continue
        tag = '--' if c is None else ('ok' if TRIMMED[0] <= c <= TRIMMED[1] else ('DOWN' if c > TRIMMED[1] else 'UP'))
        parts.append(f'CH{ch} {"  -  " if c is None else f"{c:5.1f}"}% {tag:4s}')
    return '  '.join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--live', action='store_true')
    ap.add_argument('--ch', type=int, choices=CHANNELS, help='one channel only (required with --live)')
    args = ap.parse_args()
    scope = rigol_mho.RigolScope(rigol_mho.open_transport('/dev/usbtmc0'))
    mux = MuxController(boards.find('mux'))
    mux.connect()
    try:
        setup(scope, mux, boards.find('cut'))
        channels = (args.ch,) if args.ch else CHANNELS
        if not args.live:
            print(line(read(scope, channels, mux)))
            return 0
        if not args.ch:
            ap.error('--live needs --ch')
        route_alone(mux, args.ch)
        print(f'Trim CH{args.ch} to {TRIMMED[0]:g}-{TRIMMED[1]:g} % (DOWN: turn the trimmer down; UP: up). Ctrl-C to stop.')
        while True:
            print('\r' + line(read(scope, channels)), end='', flush=True)
            time.sleep(0.3)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        mux.disable_all()
        mux.close()
        scope.close()


if __name__ == '__main__':
    sys.exit(main())
