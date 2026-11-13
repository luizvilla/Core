## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
## @file console.py
## @brief Firmware serial console for bench sessions: time-stamped log, telemetry parsing, menu keys,
##        firmware scope dumps (key r).
## @par Log format (console.txt in the session campaign)
##   <ISO time> < <received line>     every line the board prints
##   <ISO time> > <key>               every key sent (by the app or bench.py)
## @note printk drops characters under load: a telemetry line is used only when every field parses
##   (TELEMETRY), so a garbled line is kept in the log but never judged.
"""Serial console of the Spin: one reader thread, a log file, parsed telemetry and key history."""
from __future__ import annotations

import datetime as dt
import re
import threading
import time
from pathlib import Path

# The short telemetry line of src/main.cpp (every 500 ms).
TELEMETRY = re.compile(
    r'st (?P<st>\d+) inv (?P<inv>\d+) rly (?P<rly>\d\d) VAC\s*(?P<VAC>-?[\d.]+) IAC\s*(?P<IAC>-?[\d.]+) '
    r'Vbus\s*(?P<Vbus>-?[\d.]+) Id\s*(?P<Id>-?[\d.]+) Iq\s*(?P<Iq>-?[\d.]+) ok (?P<ok>\d+) flt (?P<flt>\d+) '
    r'off\s*(?P<off>-?[\d.]+)% load (?P<load>\d+)(?: lost (?P<lost>\d+))?(?: ff (?P<ff>\d))?(?: kp (?P<kp>[\d.]+) ti (?P<ti>[\d.]+))?(?: dt (?P<dt>\d+))?(?: ffn (?P<ffn>\d))?(?: dcl (?P<dcl>\d) vdc (?P<vdc>-?[\d.]+))?'
    r'(?: adt (?P<adt>[\d.]+) iacm (?P<iacm>-?[\d.]+))?')
# Names from docs/testing/bench/inverter-session.md, Telemetry.
# Stage 5 (VAC_FOLLOW, step 1d) was removed by ADR 0007; 6 and 7 kept their numbers.
STAGES = {'0': 'OFF', '1': 'SETTLE', '2': 'OPEN_LOOP', '3': 'CONNECT', '4': 'INJECT',
          '6': 'FOLLOW_CONNECT', '7': 'BUS_LOOP'}
INV_STATES = {'0': 'OFF', '1': 'VOLTAGE_SYNC', '2': 'GRID_CONNECT', '3': 'GRID_CONNECTED', '4': 'RUN',
              '5': 'STOPPING', '6': 'DISCONNECT', '7': 'FAULT', '8': 'PWM_SETTLE', '9': 'PWM_WAIT'}


def parse_telemetry(text: str) -> dict | None:
    m = TELEMETRY.search(text)
    return {k: v for k, v in m.groupdict().items() if v is not None} if m else None


def read_log(path: Path) -> tuple[list, list]:
    """@brief (rx lines [(t, text)], keys [(t, key)]) from a console.txt."""
    rx, keys = [], []
    if not path.exists():
        return rx, keys
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        parts = line.split(' ', 2)
        if len(parts) < 3:
            continue
        try:
            t = dt.datetime.fromisoformat(parts[0]).timestamp()
        except ValueError:
            continue
        (rx if parts[1] == '<' else keys if parts[1] == '>' else []).append((t, parts[2]))
    return rx, keys


class Console:
    """@brief Owns the serial port: reads lines into memory and console.txt; sends keys."""

    def __init__(self, port: str | None, log: Path, dump_dir: Path, on_dump=None, baud: int = 115200):
        self.port, self.baud, self.log, self.dump_dir, self.on_dump = port, baud, log, dump_dir, on_dump
        self.rx, self.keys = read_log(log)       # history survives an app restart
        self.lock = threading.Lock()
        self.ser = None
        self.error = ''
        self._dump: list[str] | None = None
        self._stop = threading.Event()
        if port:
            threading.Thread(target=self._run, daemon=True).start()

    @property
    def connected(self) -> bool:
        return self.ser is not None

    def _open(self):
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.error = ''

    def _write_log(self, t: float, mark: str, text: str) -> None:
        with self.log.open('a', encoding='utf-8') as f:
            f.write(f'{dt.datetime.fromtimestamp(t).isoformat(timespec="milliseconds")} {mark} {text}\n')

    def _run(self) -> None:
        buf = b''
        while not self._stop.is_set():
            try:
                if self.ser is None:
                    self._open()
                chunk = self.ser.read(4096)
            except Exception as ex:  # noqa: BLE001  (unplugged board, port busy): retry
                self.error = str(ex)
                try:
                    self.ser and self.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.ser = None
                time.sleep(1.0)
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._line(raw.decode('utf-8', errors='replace').rstrip('\r'))

    def _line(self, text: str) -> None:
        t = time.time()
        if self._dump is not None:  # firmware scope dump: kept apart from the log (thousands of hex lines)
            if 'end record' in text:
                self._save_dump(t)
            else:
                self._dump.append(text)
            return
        if 'begin record' in text:
            self._dump = []
        with self.lock:
            self.rx.append((t, text))
        self._write_log(t, '<', text)

    def _save_dump(self, t: float) -> None:
        lines, self._dump = self._dump, None
        self.dump_dir.mkdir(parents=True, exist_ok=True)
        stem = self.dump_dir / f'{dt.datetime.fromtimestamp(t):%H%M%S}_firmware_scope'
        stem.with_suffix('.txt').write_text('\n'.join(lines) + '\n')
        csv = decode_dump(lines, stem.with_suffix('.csv'))
        self._write_log(t, '<', f'[firmware scope saved: {stem.name}.txt, {len(lines)} lines]')
        if self.on_dump:
            self.on_dump(t, stem.with_suffix('.txt'), csv)

    def send(self, key: str) -> None:
        if self.ser is None:
            raise RuntimeError(f'serial port not connected ({self.error or self.port})')
        self.ser.write(key.encode())
        t = time.time()
        with self.lock:
            self.keys.append((t, key))
        self._write_log(t, '>', key)

    def latest(self) -> dict | None:
        """@brief Newest complete telemetry line, with its time."""
        with self.lock:
            for t, text in reversed(self.rx[-50:]):
                d = parse_telemetry(text)
                if d:
                    return {'t': t, **d}
        return None

    def tail(self, n: int = 200) -> list:
        with self.lock:
            out = [(t, '<', x) for t, x in self.rx[-n:]] + [(t, '>', k) for t, k in self.keys[-n:]]
        return sorted(out)[-n:]

    def window(self, t0: float, t1: float) -> tuple[list, list]:
        with self.lock:
            return ([(t, x) for t, x in self.rx if t0 <= t <= t1], [(t, k) for t, k in self.keys if t0 <= t <= t1])


# Firmware scope (main.cpp): 256 samples; 2 kHz, or 10 kHz with key v in the grid stages only. Since 2026-11-13 the
# firmware prints 'scope: 256 samples, trigger at sample 26, 2000 Hz' before each dump (trigger at 10 %). Older
# firmware printed nothing and had the trigger at 50 %: the first sample where it held was sample 129.
FW_HEADER = re.compile(r'scope: (\d+) samples, trigger at sample (\d+), (\d+) Hz')
FW_TRIGGER_LEGACY = 129
FW_GRID_STAGES = {'3', '4', '6', '7'}


def fw_timebase(rx: list, t_dump: float) -> tuple[int, int, str]:
    """@brief (rate in Hz, trigger sample, why) of a firmware scope dump: from the header the firmware printed just
    before it, else the rate guessed from the console and the trigger of the older firmware."""
    for t, x in reversed(rx):
        if t > t_dump:
            continue
        if t < t_dump - 30:
            break
        m = FW_HEADER.search(x)
        if m:
            return int(m.group(3)), int(m.group(2)), 'printed by the firmware with the dump'
    rate, why = fw_rate_guess(rx, t_dump)
    return rate, FW_TRIGGER_LEGACY, why + f'; trigger at sample {FW_TRIGGER_LEGACY} (firmware before the 10 % trigger)'


def fw_rate_guess(rx: list, t_dump: float) -> tuple[int, str]:
    """@brief Sample rate of a firmware scope dump from the console lines before it: 10 kHz when the last
    'scope in CONNECT/INJECT' message said 10 kHz and the stage was a grid stage, else 2 kHz.
    @return (rate in Hz, why)."""
    fast, stage = False, None
    for t, x in rx:
        if t > t_dump:
            break
        if 'scope in CONNECT/INJECT:' in x:
            fast = '10 kHz' in x
        d = parse_telemetry(x)
        if d:
            stage = d['st']
    if fast and stage in FW_GRID_STAGES:
        return 10000, f'10 kHz: v was on and the stage was {stage} ({STAGES.get(stage, "?")}) before the dump'
    return 2000, ('2 kHz: ' + (f'stage {stage} ({STAGES.get(stage, "?")}) is not a grid stage' if fast
                               else 'v off (default rate)'))


def parse_dump(lines: list[str]) -> tuple[list, list] | None:
    """@brief Firmware scope dump → (channel names, rows oldest first), or None."""
    import struct
    names, idx, vals = None, None, []
    for ln in (x.strip() for x in lines):
        if not ln:
            continue
        if ln.startswith('#'):
            if names is None:
                names = [n.strip() for n in ln[1:].split(',') if n.strip()]
            else:  # ScopeMimicry prints the last written index as '## <n>'
                try:
                    idx = int(ln.lstrip('#').strip())
                except ValueError:
                    pass
            continue
        try:
            vals.append(struct.unpack('>f', bytes.fromhex(ln))[0])
        except ValueError:
            continue
    if not names or not vals:
        return None
    n = len(names)
    rows = [vals[i:i + n] for i in range(0, len(vals) - len(vals) % n, n)]
    if idx is not None and rows:
        k = (idx + 1) % len(rows)
        rows = rows[k:] + rows[:k]
    return names, rows


def decode_dump(lines: list[str], csv_path: Path) -> Path | None:
    """@brief Firmware scope dump → CSV (same format as owntech/scripts/monitor/filter_recorded_datas.py:
    '#names,' line, '## <last written index>' line, then one big-endian float32 in hex per line). The rows are
    rotated so the oldest sample comes first (the trigger sample is given by fw_timebase). @return the CSV, or None."""
    got = parse_dump(lines)
    if not got:
        return None
    names, rows = got
    csv_path.write_text(','.join(names) + '\n' + '\n'.join(','.join(f'{v:.7g}' for v in r) for r in rows) + '\n')
    return csv_path
