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
| `test_connection.m` | Standalone smoke test — finds the board, opens it, and confirms `V1`/`V2` measurements can be read back. See "Test sequence" below. |
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

## Test sequence

Goal: verify, independently of the full 200-frame demo, that the MATLAB port can (a) find the
board and (b) retrieve valid measurements from it. This is what `test_connection.m` should do:

1. **Discover the port** — call `findShieldDevicePort()`. Pass/fail: returns exactly one port
   whose PnP ID matches `VID_2FE3&PID_0101` (or, on the manual fallback path, the user-selected
   port responds at all in step 2). Fail if zero or more than one candidate port is found.
2. **Open the device** — construct `ShieldDevice(port)`. Pass/fail: `serialport` opens without
   error at 115200-8-N-1; no exception thrown.
3. **Reach `POWER_ON`** — send `IDLE`, `BUCK LEG1 ON`, `LEG LEG1 ON`,
   `REFERENCE LEG1 V1 5`, `POWER_ON` (the same setup subsequence `comm_script.m` uses before
   its main loop, restricted to `LEG1` since that's all this smoke test needs). Pass/fail: each
   `sendCommand` write completes without a serial timeout/error.
4. **Read measurements** — call `getMeasurement('V1')` and `getMeasurement('V2')` 10 times in a
   loop (roughly 1 read/second is enough given the ~100 ms telemetry rate). Pass/fail:
   - Every call returns within the 2 s serial timeout (no hang).
   - Every returned value is a finite double (not `NaN`, not empty, not a parse error) —
     confirms the 16-field-line filter in `getLine`/`getMeasurement` is correctly discarding
     interleaved debug output and locking onto real telemetry frames.
   - `V1` values are in a plausible range for the bench setup (roughly 0–15 V given the demo's
     reference range) rather than garbage from a mis-parsed field index.
5. **Park the board** — send `IDLE` (in a `try`/`catch` `finally`-equivalent so this always
   runs, matching `comm_script.m`'s cleanup). Pass/fail: command sent even if step 4 threw.

Run this script manually with the board connected before relying on `comm_script.m` — it
isolates connection/parsing bugs from the plotting/animation loop, which is harder to debug
interactively.

## No-hardware validation procedure

Before the real-hardware checkpoint in Step 3, each step should be validated as far as
possible without the board attached. This machine has MATLAB on `PATH`
(`matlab -batch "..."` runs a script non-interactively and exits) but no `socat`, so the
harness below uses Python's built-in `pty` module to fake a serial device instead. This is the
same procedure used to validate `ShieldDevice.m` in Step 1 — reuse it for Steps 2 and 4.

**Two layers, run in this order:**

1. **Static check** — catches syntax/undefined-variable/style issues before ever opening a
   port:

   ```bash
   matlab -batch "result = checkcode('/path/to/File.m'); \
     if isempty(result), disp('CHECKCODE: no issues'); \
     else, for i=1:numel(result), fprintf('CHECKCODE: line %d: %s\n', result(i).line, result(i).message); end; end"
   ```

2. **Behavioral check** — exercises the real code path against a virtual serial port:
   - A Python helper (`fake_board.py`) opens a pty pair with `pty.openpty()`, writes the slave
     device path (e.g. `/dev/pts/5`) to a file so MATLAB can read it, then loops: logs every
     byte it receives on the master end (with a timestamp, so chunking/pacing can be checked),
     and periodically writes a synthetic response — e.g. a bogus non-16-field "debug" line
     followed by a well-formed 16-field telemetry frame with known values planted at specific
     indices.
   - A MATLAB wrapper function (`test_<thing>.m`) reads the port path from that file,
     exercises the class/function under test (constructs `ShieldDevice`, calls `sendCommand`/
     `getMeasurement`/etc., wrapped in `try`/`catch` so failures are reported instead of
     aborting the batch run), and `fprintf`s results in an easily `grep`-able form
     (`MSG1:...`, `V1:...`, `ERRTEST:OK:...`).
   - Run the Python harness in the background, wait for its port-path file to appear, then run
     `matlab -batch "test_<thing>('/path/to/port.txt')"`, then kill the harness and inspect its
     byte log alongside MATLAB's stdout.
   - Put both files under the scratchpad/temp directory, not in `src/matlab/` — they are test
     scaffolding for this validation procedure, not part of the shipped `.m` files.

**What this can and cannot prove:**

- Can prove: exact wire-format strings, chunk sizes and inter-chunk/inter-command timing
  (from the logged receive timestamps), correct parsing/index-mapping of measurement frames,
  correct rejection of malformed/interleaved lines, correct error handling on bad input —
  i.e. everything in the "Protocol reference" section above.
- Cannot prove: real USB enumeration or VID/PID matching (a pty has no USB VID/PID, so
  `findShieldDevicePort`'s autodetection logic itself can only be validated against real
  hardware or by unit-testing its string-matching against fixture strings — its *manual
  fallback* prompt path, which just wraps `serialportlist`, can still be exercised), and
  anything about the real firmware's actual timing/format/quirks. That gap is exactly what
  Step 3's `test_connection.m` run against the physical board is for — treat a pass here as
  "implementation matches the documented protocol," not as "verified against hardware."

Record the result in the corresponding Work-sequence step below (checkboxes + a dated
"Verified" note), the same way Step 1 is recorded, so it's visible without re-deriving it.

## Commit sequence

This README documents the plan; a follow-up implementation task should land the `.m` files in
small, independently-reviewable commits rather than one large drop:

1. `ShieldDevice.m` — `feat(matlab): add ShieldDevice class for Twist serial protocol`
2. `findShieldDevicePort.m` — `feat(matlab): add board auto-discovery by VID/PID`
3. `test_connection.m` — `test(matlab): add smoke test for board discovery and measurement read-back`
   — run this against real hardware before continuing; it's the checkpoint that the two
   commits above actually work end-to-end.
4. `comm_script.m` — `feat(matlab): add MATLAB port of comm_script.py demo loop`
5. Any README corrections discovered while implementing/testing — `docs(matlab): correct README per implementation findings`

Each commit should be buildable/runnable on its own (e.g. commit 1 alone lets you construct a
`ShieldDevice` even before discovery is automated). Do not squash the smoke test in with
`comm_script.m` — it needs to keep working standalone as a fast way to debug hardware
connectivity issues separately from the plotting loop.

## Work sequence (resumable, one block per commit)

Each block below is self-contained: precondition, checklist, definition of done, and the
commit to make once done. **To resume after a break, run `git log --oneline -- src/matlab/`,
find the highest-numbered step whose commit message (from the table above) already exists,
and start at the next block.** Tick the checkboxes off as you go within a session; if state is
lost, the definition-of-done line tells you how to re-derive whether a step is actually
finished by inspecting the repo/hardware rather than trusting memory.

### Step 1 — `ShieldDevice.m`

- **Precondition**: none (first step).
- **Resume check**: `git log --oneline -- src/matlab/ShieldDevice.m` — if it returns a commit,
  this step is done; skip to Step 2.
- **Do**:
  - [x] Create `classdef ShieldDevice` with constructor `ShieldDevice(port)` opening a
        `serialport` at 115200-8-N-1, 2 s timeout.
  - [x] Add the 16-field TWIST index map (`D1..RS`, see table above) as a property.
  - [x] Implement `sendMessage(msg)`: write in 10-char chunks with 0.1 s pauses, then `\r\n`.
  - [x] Implement `sendCommand(action, varargin)` using the command format table above,
        followed by the 0.2 s settle delay.
  - [x] Implement `getLine()`: read one line, split on `:`.
  - [x] Implement `getMeasurement(name)`: reset input buffer, loop `getLine()` discarding
        anything that isn't 16 fields after stripping `{`/`}`, return the field as `double`.
- **Definition of done**: in MATLAB, `d = ShieldDevice(anyValidPort); d.sendCommand('IDLE')`
  runs without error (board does not need to be attached yet for this step — class
  construction and message formatting can be unit-checked with a loopback port or by
  inspecting the built string before it's written).
  - **Verified 2026-07-13** without real hardware, using a Python `pty`-based virtual serial
    loopback (one end opened by `ShieldDevice`, the other end logging raw bytes and injecting
    a synthetic 16-field telemetry frame): `checkcode` reported no issues; `sendCommand('LEG',
    'LEG1','ON')` produced exactly `s_LEG1_l_on`, transmitted as `s_LEG1_l_o`/`n`/`\r\n` with
    ~0.1 s gaps (10-char chunking confirmed) and a ~0.2 s gap before the next command (settle
    delay confirmed); `sendCommand('REFERENCE','LEG1','V1',5)` produced
    `s_LEG1_r_V1_5.00000`; `getMeasurement('V1')`/`getMeasurement('V2')` correctly skipped an
    interleaved non-16-field debug line and returned the exact planted values, confirming the
    field-count filter and index map; `sendCommand('BOGUS')` correctly threw
    `ShieldDevice:InvalidAction`. **Still outstanding**: confirmation against the real board
    (USB enumeration, actual firmware telemetry timing/format) — that's what Step 3's smoke
    test is for.
- **Commit**: `feat(matlab): add ShieldDevice class for Twist serial protocol`

### Step 2 — `findShieldDevicePort.m`

- **Precondition**: Step 1 committed.
- **Resume check**: `git log --oneline -- src/matlab/findShieldDevicePort.m`.
- **Do**:
  - [ ] Implement the Linux path: scan `/dev/serial/by-id/*` for `2fe3`/`0101`.
  - [ ] Implement the Windows path: `wmic path Win32_PnPEntity` (or registry) lookup for
        `VID_2FE3&PID_0101`, resolve to a `COMx` port.
  - [ ] Implement the manual fallback: list `serialportlist("available")`, prompt for a
        choice, when autodetection finds zero or >1 candidates.
- **Definition of done**: with the board plugged in, `findShieldDevicePort()` returns exactly
  one port string, and it matches what `serialportlist("available")` shows for the board.
  - **Pre-hardware pass** (see "No-hardware validation procedure" above): `checkcode` clean;
    the manual-fallback prompt path exercised against `serialportlist("available")` on this
    machine (with no board attached, so it should list whatever's here and let you pick/cancel
    without erroring); the Linux/Windows string-matching branches exercised against fixture
    PnP-ID/by-id strings rather than a real device, since a pty has no VID/PID. This is *not*
    a substitute for the real check above — record both separately.
- **Commit**: `feat(matlab): add board auto-discovery by VID/PID`

### Step 3 — `test_connection.m`

- **Precondition**: Steps 1–2 committed; board flashed and connected via USB (see
  Prerequisites).
- **Resume check**: `git log --oneline -- src/matlab/test_connection.m`.
- **Do**: implement the 5 numbered steps from "Test sequence" above
  (discover → open → reach `POWER_ON` → read 10x → park with `IDLE`), printing pass/fail for
  each to the console.
  - [ ] Step 1 (discover) implemented and passing.
  - [ ] Step 2 (open) implemented and passing.
  - [ ] Step 3 (reach `POWER_ON`) implemented and passing.
  - [ ] Step 4 (read 10x, finite + plausible-range check) implemented and passing.
  - [ ] Step 5 (park with `IDLE`, runs even on error) implemented and passing.
- **Definition of done**: running `test_connection.m` against real hardware prints 5/5 pass
  and returns the board to `IDLE`. **This is the hard checkpoint** — do not start Step 4 until
  this genuinely passes against the physical board, since it's what proves Steps 1–2 work
  end-to-end rather than just compiling.
- **Commit**: `test(matlab): add smoke test for board discovery and measurement read-back`

### Step 4 — `comm_script.m`

- **Precondition**: Step 3 committed and passing against real hardware.
- **Resume check**: `git log --oneline -- src/matlab/comm_script.m`.
- **Do**:
  - [ ] Port the setup sequence (`IDLE` → `BUCK`×2 → `LEG`×2 → `REFERENCE LEG1 V1 5` →
        `POWER_ON`), reusing `ShieldDevice`/`findShieldDevicePort`.
  - [ ] Implement the 200-frame loop: triangular reference (5, +0.5, wrap at 15), send
        `REFERENCE` for both legs, 10 ms extra sleep, read `V1`/`V2`.
  - [ ] Implement the live plot with `animatedline`/`drawnow limitrate`, resetting the window
        every 200 frames.
  - [ ] Wrap the loop in `try`/`catch` that always sends `IDLE` on exit.
- **Definition of done**: running `comm_script.m` against real hardware shows a live-updating
  V1/V2 plot tracking the triangular reference, and closing the figure (or Ctrl+C) leaves the
  board in `IDLE`.
  - **Pre-hardware pass** (see "No-hardware validation procedure" above): `checkcode` clean;
    extend `fake_board.py` to answer the full setup sequence (not just `LEG`/`REFERENCE`) and
    to stream continuously-updating telemetry so the 200-frame loop's reference ramp/wrap
    logic and repeated `getMeasurement` calls can be checked against known planted values over
    many frames; run under `matlab -batch` with `-nodisplay` and confirm it completes 200
    frames and sends the final `IDLE` without error. Note: batch mode has no figure window, so
    this only proves the control/data-flow logic, not that the plot actually renders correctly
    — visually confirming the live plot is real-hardware-only, part of the Step 4 real check.
- **Commit**: `feat(matlab): add MATLAB port of comm_script.py demo loop`

### Step 5 — README corrections

- **Precondition**: Steps 1–4 done.
- **Do**: fix any discrepancy discovered during implementation/testing between this README and
  the actual working `.m` files (e.g. a timing constant that needed tweaking on real hardware).
- **Definition of done**: README matches the shipped code.
- **Commit**: `docs(matlab): correct README per implementation findings`

## Next steps

Progress against the Work sequence above:

- [x] **Step 1** — `ShieldDevice.m` implemented, `checkcode`-clean, and verified against the
  no-hardware pty-loopback procedure (see Step 1's "Verified" note). Real-hardware
  confirmation still outstanding — folded into Step 3.
- [ ] **Step 2** — `findShieldDevicePort.m`. **This is the next action.** Implement the
  Linux/Windows autodetection plus manual fallback described above, run the pre-hardware pass
  from "No-hardware validation procedure," then commit.
- [ ] **Step 3** — `test_connection.m`, then run it against the real board. This is the hard
  checkpoint: nothing before it has touched real hardware, and nothing after it should be
  trusted until it passes.
- [ ] **Step 4** — `comm_script.m`.
- [ ] **Step 5** — reconcile any README/implementation drift found along the way.

If resuming cold: run `git log --oneline -- src/matlab/` to see which of the files above
already have commits, match that against the checkboxes here and in the corresponding Work
sequence block, and continue from the first unchecked item.
