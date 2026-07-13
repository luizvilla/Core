# MATLAB Communication Protocol (plan)

This document is the design plan for a MATLAB port of [`../comm_script.py`](../comm_script.py).
It describes the exact serial protocol the Python script uses to drive a Twist 1.4.1 board and
the file layout a follow-up implementation task will create in this folder. **No `.m` code is
part of this change** — this README is the reference the implementation must match.

## Overview

`comm_script.py` puts a Twist board through a fixed setup sequence (idle → configure both legs
in buck mode → set an initial voltage reference → power on), then runs a real-time loop that
ramps the voltage reference on `LEG1`/`LEG2` and plots the measured `V1`/`V2` values live. The
MATLAB port should reproduce this exactly: same wire protocol, same command sequence, same
measurement parsing, same live plot behaviour — for users who prototype in MATLAB/Simulink
instead of Python.

## Support script files needed

A follow-up task should create the following files in `src/matlab/`:

| File | Purpose |
|---|---|
| `ShieldDevice.m` | `classdef` wrapping a `serialport` object. Methods: `sendCommand`, `sendMessage`, `getMeasurement`, `getLine`. Constructor defaults match `Shield_Class.__init__`: 115200 baud, 8 data bits, no parity, 1 stop bit, 2 s timeout. Holds the 16-field TWIST index map (see below) as a property. |
| `findShieldDevicePort.m` | Device discovery helper, analogue of `find_devices.py`. See caveat below — MATLAB has no direct cross-platform VID/PID query like pyserial's `list_ports`, so this does best-effort OS-specific autodetection with a manual fallback. |
| `comm_script.m` | Top-level script reproducing the exact command sequence and the real-time plot loop from `comm_script.py`. |
| `README.md` | This document. |

### `findShieldDevicePort.m` — device discovery caveat

`serialportlist` in MATLAB lists available serial ports but does not expose USB VID/PID the
way pyserial's `serial.tools.list_ports.comports()` does. The plan is a best-effort
autodetect with manual fallback:
- **Linux**: parse `/dev/serial/by-id/*` symlinks for the string `2fe3` / `0101` (VID/PID are
  embedded in the udev-generated symlink name for USB-CDC devices).
- **Windows**: query `wmic path Win32_PnPEntity` (or the registry) for a PnP device ID
  containing `VID_2FE3&PID_0101`, then resolve it to a COM port.
- **Fallback (any OS)**: if autodetection fails, list `serialportlist("available")` and prompt
  the user to pick one.

## Protocol reference (ground truth for the implementation)

Reverse-engineered from `Shield_Class.py`/`find_devices.py` (the `comm_protocol` library
pinned in [`../app.ini`](../app.ini) at `python_twist_comm_protocol`, branch `power_tuesday`)
and cross-checked against the firmware-side parser (`comm_protocol.cpp`, `initial_handle` /
`console_getchar` loop referenced in [`../README.md`](../README.md)).

### Connection

| Parameter | Value |
|---|---|
| Interface | USB-CDC virtual COM port |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Read timeout | 2 s |
| USB VID / PID | `0x2FE3` / `0x0101` |

### Sending a command

1. Build the ASCII command string (see table below).
2. Write it in **10-character chunks**, sleeping **0.1 s** between chunks (the firmware reads
   one character at a time via `console_getchar()`, so this paces transmission to match).
3. Write a `\r\n` terminator.
4. Sleep an additional **0.2 s** settle delay before sending anything else (matches
   `sendCommand`'s default `delay=0.2`).

There is **no checksum or binary framing** — this is a plain ASCII, underscore-delimited,
newline-terminated protocol.

### Command format table

| Action | Args | Wire format |
|---|---|---|
| `IDLE` | — | `d_i` |
| `POWER_OFF` | — | `d_f` |
| `POWER_ON` | — | `d_o` |
| `LEG` | leg, state | `s_{LEG}_l_{state}` |
| `CAPA` | leg, state | `s_{LEG}_c_{state}` |
| `DRIVER` | leg, state | `s_{LEG}_v_{state}` |
| `BUCK` | leg, state | `s_{LEG}_b_{state}` |
| `BOOST` | leg, state | `s_{LEG}_t_{state}` |
| `REFERENCE` | leg, variable, value | `s_{LEG}_r_{VARIABLE}_{value:.5f}` |
| `DUTY` | leg, value | `s_{LEG}_d_{value:.5f}` |
| `CALIBRATE` | variable, gain, offset | `k_{VARIABLE}_g_{gain:.8f}_o_{offset:.8f}` |

`{LEG}`/`{VARIABLE}` are upper-cased, `{state}` is lower-cased, matching
`leg.upper()`/`state.lower()` in the Python source.

### Reading a measurement

1. Flush/reset the serial input buffer.
2. Read a line (terminated by `\n`).
3. Split it on `:`.
4. Strip `{` and `}` characters from each field (the firmware wraps the last 4 fields of the
   telemetry frame in `{}`).
5. If the line does not split into **exactly 16 fields**, discard it and read the next line.
   Debug/log text can be interleaved on the same UART with telemetry frames, so this
   field-count check is the only framing the protocol has — there is no fixed line length or
   header to rely on.
6. Once a 16-field line is found, parse the field at the index below as a float.

### TWIST field index map (16 fields)

| Field | Index | Field | Index |
|---|---|---|---|
| D1 | 0 | D2 | 5 |
| V1 | 1 | I2 | 6 |
| I1 | 2 | V2 | 7 |
| M1 | 3 | M2 | 8 |
| T1 | 4 | T2 | 9 |
| VH | 10 | IH | 11 |
| AN | 12 | CE | 13 |
| CR | 14 | RS | 15 |

## `comm_script.m` flow (must match `comm_script.py` exactly)

1. Discover the board port (`findShieldDevicePort`) and open a `ShieldDevice`.
2. Setup sequence, each command followed by the chunked-write + 0.2 s delay described above:
   - `IDLE`
   - `BUCK LEG1 ON`
   - `BUCK LEG2 ON`
   - `LEG LEG1 ON`
   - `LEG LEG2 ON`
   - `REFERENCE LEG1 V1 5`
   - `POWER_ON`
3. Real-time loop, 200 frames:
   - Increment a triangular reference: start at 5, step `+0.5` per frame, wrap back to 5 when
     it reaches 15.
   - Send `REFERENCE LEG1 V1 <ref>` and `REFERENCE LEG2 V2 <ref>`.
   - Sleep an extra 10 ms (on top of the 0.2 s built into the command send).
   - Read `V1` and `V2` measurements and append them to a live plot.
   - Every 200 frames, clear the plot and re-base the sliding time axis.
4. On exit — normal completion, error, or figure close — always send `IDLE` to park the board
   (MATLAB `try`/`catch`, the analogue of the Python `try`/`finally`).

## Differences / porting notes

- Python's unused imports in `comm_script.py` (`xmlrpc.client`, `numpy`) are dropped — they
  aren't exercised by the script's logic.
- Python f-string numeric formatting (`{value:.5f}`, `{gain:.8f}`) becomes MATLAB `sprintf`
  format specifiers (`%.5f`, `%.8f`).
- Use MATLAB's modern `serialport` object (Instrument Control Toolbox, R2019b+) rather than
  the legacy `serial`/`instrfind` API.
- Python's `matplotlib.animation.FuncAnimation` becomes a MATLAB loop using `animatedline` and
  `drawnow limitrate` for the live-updating plot.

## Prerequisites

- MATLAB with Instrument Control Toolbox (required for `serialport`).
- Twist board flashed per the firmware setup in [`../README.md`](../README.md).
- Board connected via USB.

## Next steps

This README is the design plan only. `ShieldDevice.m`, `findShieldDevicePort.m`, and
`comm_script.m` are a follow-up implementation task and are not part of this change.
