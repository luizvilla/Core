# MATLAB Communication Protocol

MATLAB port of [`../comm_script.py`](../comm_script.py), driving a Twist 1.4.1 board over the
same serial protocol. **Status: implemented and verified against real hardware** — all four
`.m` files below exist, and the Work sequence section records dated evidence for each one,
including two real-hardware runs (unpowered and powered at 30 Vdc) and an interactive
confirmation that the live plot renders correctly. This document remains the protocol
reference and implementation log: the tables below are the ground truth the `.m` files were
built against, and the Work sequence is the audit trail of how each was actually verified —
keep both in sync with the code going forward, per Step 5.

## Overview

`comm_script.py` puts a Twist board through a fixed setup sequence (idle → configure both legs
in buck mode → set an initial voltage reference → power on), then runs a real-time loop that
ramps the voltage reference on `LEG1`/`LEG2` and plots the measured `V1`/`V2` values live. The
MATLAB port reproduces this: same wire protocol, same command sequence, same measurement
parsing, same live plot behaviour — for users who prototype in MATLAB/Simulink instead of
Python. One deliberate deviation is documented below (a dead-code plot-reset bug in the
original was fixed rather than reproduced); everything else matches.

## Support script files

`src/matlab/` contains:

| File | Purpose |
|---|---|
| `ShieldDevice.m` | `classdef` wrapping a `serialport` object. Methods: `sendCommand`, `sendMessage`, `getMeasurement`, `getLine`. Constructor defaults match `Shield_Class.__init__`: 115200 baud, 8 data bits, no parity, 1 stop bit, 2 s timeout. Holds the 16-field TWIST index map (see below) as a property. |
| `findShieldDevicePort.m` | Device discovery function, analogue of `find_devices.py`. See caveat below — MATLAB has no direct cross-platform VID/PID query like pyserial's `list_ports`, so this does best-effort OS-specific autodetection with a manual fallback. |
| `comm_script.m` | Function reproducing `comm_script.py`'s command sequence and real-time plot loop, with `FrameLimit`/`MaxCycles`/`EnablePlot` options added for headless/bounded verification — call it with no arguments to reproduce the original's behaviour exactly (run with a live plot until the figure is closed). |
| `test_connection.m` | Standalone smoke test — finds the board, opens it, and confirms `V1`/`V2` measurements can be read back. See "Test sequence" below. |
| `README.md` | This document. |

### `findShieldDevicePort.m` — device discovery caveat

`serialportlist` in MATLAB lists available serial ports but does not expose USB VID/PID the
way pyserial's `serial.tools.list_ports.comports()` does. Implemented as a best-effort
autodetect with manual fallback:

- **Linux**: walk `/sys/class/tty/<tty>/device`, climbing parent directories until an
  `idVendor`/`idProduct` file pair is found, and compare their contents to the target VID/PID
  (passed in via `VendorID`/`ProductID` name-value arguments, default `2fe3`/`0101`). This
  reads the authoritative kernel-reported USB IDs directly rather than parsing
  `/dev/serial/by-id/*` symlink names — those names are built from USB manufacturer/product
  *strings*, not VID/PID hex, so they weren't a reliable match target and were dropped in
  favor of the sysfs approach during implementation. Verified against real hardware — see
  Step 2 below.
- **Windows**: query `wmic path Win32_PnPEntity` for a PnP device ID containing
  `VID_<VendorID>&PID_<ProductID>` (same default `2fe3`/`0101`), then resolve it to a COM
  port. Implemented per this description; **unverified** — no Windows machine was available
  during implementation.
- **Fallback (any OS)**: if autodetection fails, list `serialportlist("available")` and prompt
  the user to pick one. `findShieldDevicePort` also accepts an `'Interactive', false`
  name-value pair that skips the prompt — auto-picks the sole candidate if exactly one port is
  available, otherwise errors — so the fallback path itself can be exercised in a non-interactive
  (e.g. batch/CI) context.
- **Note on the default PID**: real-hardware testing (Step 2/3) found the same physical board
  reporting PID `0x0100` in one session and `0x0101` (the default used here, matching
  `comm_script.py`'s hardcoded value) in another. Don't assume auto-detection will find your
  board on the first try — check `cat /sys/class/tty/ttyACM*/device/../idProduct` (Linux) or
  `lsusb` first, and pass `'ProductID', '...'` explicitly if it differs.

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
| USB VID / PID | `0x2FE3` / `0x0101` (documented default — seen to vary on real hardware, see the device discovery caveat above) |

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
   - Every 200 frames, clear the plot and re-base the sliding time axis. **Implementation
     note**: `comm_script.py`'s own version of this reset is dead code —
     `FuncAnimation(..., frames=range(frame_limit), ...)` only ever calls its callback with
     `frame` in `0..frame_limit-1`, so the `if frame == frame_limit:` clear branch can never
     run, and the Python plot's data lists grow unbounded across animation repeats instead of
     ever resetting. `comm_script.m` implements the reset as originally documented here
     (working, bounded-memory behavior) rather than reproducing that bug — see "Differences /
     porting notes" below.
4. On exit — normal completion, error, or figure close — always send `IDLE` to park the board.
   Implemented with MATLAB's `onCleanup`, not `try`/`catch`: `onCleanup` fires on normal
   return, a thrown error, *and* a user Ctrl+C interrupt, which a plain `try`/`catch` would not
   reliably catch — a closer match to Python's `try`/`finally` guarantee than `try`/`catch` is.

## Differences / porting notes

- Python's unused imports in `comm_script.py` (`xmlrpc.client`, `numpy`) are dropped — they
  aren't exercised by the script's logic.
- Python f-string numeric formatting (`{value:.5f}`, `{gain:.8f}`) becomes MATLAB `sprintf`
  format specifiers (`%.5f`, `%.8f`).
- Use MATLAB's modern `serialport` object (Instrument Control Toolbox, R2019b+) rather than
  the legacy `serial`/`instrfind` API.
- Python's `matplotlib.animation.FuncAnimation` becomes a MATLAB loop using `animatedline` and
  `drawnow limitrate` for the live-updating plot.
- `comm_script.m` fixes a dead-code bug in `comm_script.py`'s plot-reset logic (see
  `comm_script.m` flow, step 3, above) rather than reproducing it, since the intent (a
  bounded, resetting sliding window) is what's documented and useful, not the unreachable
  branch.
- `comm_script.m` is a function with name-value options (`FrameLimit`, `MaxCycles`,
  `EnablePlot`, plus the `findShieldDevicePort` pass-throughs), not a bare top-level script
  like `comm_script.py`. `MaxCycles`/`EnablePlot` exist specifically to make headless,
  bounded-duration verification runs possible (see Step 4's "Verified" note) — calling
  `comm_script()` with no arguments reproduces the original's "run with a live plot until the
  figure is closed" behavior.

## Prerequisites

- MATLAB with Instrument Control Toolbox (required for `serialport`).
- Twist board flashed per the firmware setup in [`../README.md`](../README.md).
- Board connected via USB.

## Test sequence

Goal: verify, independently of the full 200-frame demo, that the MATLAB port can (a) find the
board and (b) retrieve valid measurements from it. This is what `test_connection.m` should do:

1. **Discover the port** — call `findShieldDevicePort()`. Pass/fail: returns exactly one port
   whose PnP ID matches the target VID/PID (default `VID_2FE3&PID_0101`, but see the device
   discovery caveat above — real hardware has been observed reporting `PID_0100` in one
   session; pass `'ProductID','0100'` if auto-detection comes up empty) (or, on the manual
   fallback path, the user-selected port responds at all in step 2). Fail if zero or more than
   one candidate port is found with the PID actually in use.
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
   - A plausible-range check (`V1` roughly 0–15 V per the demo's reference range) was in the
     original plan here but was **dropped from the implementation**: it requires the board's
     DC supply to actually be connected and powered, which the Step 3 verification run
     deliberately did not do (see that section's caveat — enabling power flow into an
     unconfigured bench setup is a real electrical safety call, not one to make by default).
     With no supply connected, `V1`/`V2` read consistently negative (~−10 V / −11.8 V), which
     is expected ADC/offset behavior, not a bug. Finite-and-no-timeout is what's actually
     checked; a real range check is deferred to whoever next runs this with the bench properly
     wired per the Hardware wiring section of `../README.md`.
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

**Why a real board is unavoidable eventually**: this procedure validates the MATLAB code
against *my reconstruction* of the protocol from reading the firmware source — not against the
firmware actually running. A pty-loopback pass only proves internal consistency (the code does
what the README says), it cannot catch a case where the README/source-reading is itself wrong,
where the real board's USB-CDC enumeration behaves differently than assumed, or where actual
telemetry timing/framing has quirks not visible in source. Concretely, a real board is required
starting at:

- **Step 2** — to prove `findShieldDevicePort` actually finds VID `0x2FE3`/PID `0x0101` on this
  OS, not just that its string-matching logic is internally correct against fixtures.
- **Step 3** — the hard checkpoint: nothing before it has touched real hardware, and nothing
  after it should be trusted until it passes against the physical board.
- **Step 4** — to confirm the live plot actually renders and tracks real measurements (batch
  mode has no figure window, so the no-hardware pass for Step 4 only proves control/data-flow
  logic, never the plot itself).

No-hardware passes are a fast way to catch implementation bugs early and cheaply, but they are
additive to, not a replacement for, running against the board.

Record the result in the corresponding Work-sequence step below (checkboxes + a dated
"Verified" note), the same way Step 1 is recorded, so it's visible without re-deriving it.

## Commit sequence

The `.m` files landed in small, independently-reviewable commits rather than one large drop,
in this order (all now on the branch — see `git log --oneline -- src/matlab/`):

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
  - [x] Implement the Linux path: walk `/sys/class/tty/<tty>/device` up to `idVendor`/
        `idProduct` and compare (see caveat note above — this replaced the originally-planned
        `/dev/serial/by-id/*` string match, which doesn't reliably contain raw VID/PID hex).
  - [x] Implement the Windows path: `wmic path Win32_PnPEntity` lookup for
        `VID_2FE3&PID_0101`, resolve to a `COMx` port.
  - [x] Implement the manual fallback: list `serialportlist("available")`, prompt for a
        choice, when autodetection finds zero or >1 candidates; added a non-interactive mode
        (`'Interactive', false`) so this path is testable without blocking on stdin.
- **Definition of done**: with the board plugged in, `findShieldDevicePort()` returns exactly
  one port string, and it matches what `serialportlist("available")` shows for the board.
  - **Verified 2026-07-13**, and unusually this went beyond the planned pre-hardware pass
    because a real shield happened to be attached to this machine: `checkcode` reported no
    issues; with no VID/PID override, autodetection correctly found **zero** matches for the
    documented default PID `0x0101` and fell through to the manual-fallback path, which
    correctly errored (`ShieldDevice:AmbiguousSelection`) in non-interactive mode with 33
    ports available (mostly legacy `/dev/ttyS*`). Reading
    `/sys/class/tty/ttyACM0/device/../idVendor` and `../idProduct` directly showed the
    attached device is actually **VID `2fe3` / PID `0100`**, not `0101`. Calling
    `findShieldDevicePort('VendorID','2fe3','ProductID','0100')` correctly returned
    `/dev/ttyACM0` — a genuine positive-match test against real USB sysfs data, not a fake
    loopback. Windows path remains unverified (no Windows machine available).
  - **Open question for Step 3**: `comm_script.py` hardcodes `shield_pid = 0x0101`, which this
    README's default also used, but the real board observed here reports `0x0100`. This may be
    a board/firmware-revision difference, or `comm_script.py`'s constant may simply be stale.
    Whoever runs Step 3 should check the actual PID on their bench board (Linux:
    `cat /sys/class/tty/ttyACM*/device/../idProduct`, or `lsusb`) before assuming
    `findShieldDevicePort()`'s default will auto-detect it — pass `'ProductID','0100'`
    explicitly if needed, or fall back to manual selection, which works either way.
- **Commit**: `feat(matlab): add board auto-discovery by VID/PID`

### Step 3 — `test_connection.m`

- **Precondition**: Steps 1–2 committed; board flashed and connected via USB (see
  Prerequisites).
- **Resume check**: `git log --oneline -- src/matlab/test_connection.m`.
- **Do**: implement the 5 numbered steps from "Test sequence" above
  (discover → open → reach `POWER_ON` → read 10x → park with `IDLE`), printing pass/fail for
  each to the console.
  - [x] Step 1 (discover) implemented and passing.
  - [x] Step 2 (open) implemented and passing.
  - [x] Step 3 (reach `POWER_ON`) implemented and passing.
  - [x] Step 4 (read 10x; relaxed to a finite-value check only, see note below) implemented
        and passing.
  - [x] Step 5 (park with `IDLE`, runs even on error) implemented and passing.
- **Definition of done**: running `test_connection.m` against real hardware prints 5/5 pass
  and returns the board to `IDLE`. **This is the hard checkpoint** — do not start Step 4 until
  this genuinely passes against the physical board, since it's what proves Steps 1–2 work
  end-to-end rather than just compiling.
  - **Verified 2026-07-13 against the real attached board** (`/dev/ttyACM0`, VID `2fe3`/PID
    `0101` — the PID now matched the documented default, unlike the `0x0100` seen during Step
    2; the board may have re-enumerated or changed mode between sessions, so both values have
    been observed on this same physical unit and `findShieldDevicePort`'s optional
    `ProductID` override remains the escape hatch if it happens again). `checkcode` clean.
    All 5 checks passed against the real firmware, not a loopback: discovery found exactly one
    port, `ShieldDevice` opened it, the full `IDLE`→`BUCK`→`LEG`→`REFERENCE`→`POWER_ON`
    sequence was sent without a serial error, `getMeasurement('V1')`/`getMeasurement('V2')`
    returned 10 finite values each with no timeout, and the final `IDLE` was sent successfully.
  - **Caveat on the measurement values**: the board's DC supply was intentionally *not*
    connected/powered for this run (a deliberate call — enabling actual power flow into an
    unconfigured bench setup would be a real electrical safety risk, not just a software one).
    `V1`/`V2` read consistently around −10 V / −11.8 V, which is not a physically meaningful
    converter voltage — it's expected ADC/offset behavior with no bus voltage present. The
    plausible-voltage-range check originally planned for Step 4 was dropped in favor of a
    finite/no-timeout check only, since a real range check requires the board to actually be
    powered. **This run proves the protocol implementation (discovery, command formatting,
    frame parsing) is correct end-to-end against real firmware — it does not validate the
    analog measurement chain or control loop.** A follow-up run with the bench properly wired
    per `../README.md` (source + load) would be needed to confirm the values track a real
    voltage reference.
  - **Re-verified 2026-07-13 with the bench powered** (30 Vdc source connected, per the
    caveat above). Re-ran `test_connection.m` unchanged: 5/5 still passed.
    `getMeasurement('V1')` returned `[5.17089, 4.86114, 5.12664, 4.90539, 4.94964, 5.08239,
    4.94964, 4.99389, 4.72839, 4.86114]` — mean ≈ 4.96 V, tightly clustered around the
    `REFERENCE LEG1 V1 5` command sent in the setup sequence. `getMeasurement('V2')` returned
    values near 0 V (mean ≈ 0.2 V) as expected, since this smoke test only configures/drives
    `LEG1` — `LEG2` was never given a reference or turned on. **This closes the gap the
    unpowered run left open**: it confirms the analog measurement chain and the closed-loop
    voltage regulation actually work, not just the serial protocol. The
    plausible-voltage-range check dropped from the implementation earlier would now have
    passed for `V1`; it's still not re-added to `test_connection.m` itself since the check's
    validity depends on the bench being powered, which the script has no way to verify —
    treat this real-hardware log as the range confirmation instead of a hardcoded assertion.
- **Commit**: `test(matlab): add smoke test for board discovery and measurement read-back`

### Step 4 — `comm_script.m`

- **Precondition**: Step 3 committed and passing against real hardware.
- **Resume check**: `git log --oneline -- src/matlab/comm_script.m`.
- **Do**:
  - [x] Port the setup sequence (`IDLE` → `BUCK`×2 → `LEG`×2 → `REFERENCE LEG1 V1 5` →
        `POWER_ON`), reusing `ShieldDevice`/`findShieldDevicePort`.
  - [x] Implement the 200-frame loop: triangular reference (5, +0.5, wrap at 15), send
        `REFERENCE` for both legs, 10 ms extra sleep, read `V1`/`V2`.
  - [x] Implement the live plot with `animatedline`/`drawnow limitrate`, resetting the window
        every `FrameLimit` frames (working reset — see the dead-code note in the flow section
        above for why this isn't a literal line-for-line port of `comm_script.py`).
  - [x] Guarantee `IDLE` on exit via `onCleanup` (covers normal completion, error, figure
        close, and Ctrl+C — see "Differences / porting notes" for why this was chosen over
        `try`/`catch`).
- **Definition of done**: running `comm_script.m` against real hardware shows a live-updating
  V1/V2 plot tracking the triangular reference, and closing the figure (or Ctrl+C) leaves the
  board in `IDLE`.
  - **Verified 2026-07-13 against the real, powered board** (30 Vdc, same session as Step 3's
    powered re-verification). `checkcode` clean. Ran the full functional loop headless —
    `comm_script('EnablePlot', false, 'MaxCycles', 1)` — driving **both legs** (LEG2 exercised
    for the first time in this project) through the complete 200-frame ramp (5→15→5 V) against
    real firmware: completed in 205.4 s with no errors, consistent with the expected ~1 s/frame
    from the chunked-write/settle-delay timing documented above. Independently confirmed the
    board returned to `IDLE` afterward by raw-reading `/dev/ttyACM0` for 3 s and observing no
    telemetry — matches the firmware's documented behavior that `IDLE` stops broadcasting.
  - **Live plot visually confirmed 2026-07-13**: the headless run above proved the
    control/data-flow but not that anything actually rendered, since this environment only had
    non-interactive `matlab -batch` access. The user ran `comm_script()` with default
    arguments in an interactive MATLAB desktop session against the same powered board and
    confirmed it worked — the live V1/V2 plot renders and tracks the ramping reference as
    intended. This closes the last open gap from the earlier headless-only verification; all
    four `.m` files now have real-hardware evidence for both protocol correctness and (for
    `comm_script.m`) the visual/interactive behavior.
- **Commit**: `feat(matlab): add MATLAB port of comm_script.py demo loop`

### Step 5 — README corrections

- **Precondition**: Steps 1–4 done.
- **Do**: fix any discrepancy discovered during implementation/testing between this README and
  the actual working `.m` files (e.g. a timing constant that needed tweaking on real hardware).
  - [x] Title/intro updated from "design plan, no code yet" to reflect implemented +
        real-hardware-verified status.
  - [x] "Support script files needed" reworded from future tense to present (all four files
        exist); `comm_script.m` row updated to describe it as a function with options, not a
        bare script.
  - [x] Windows device-discovery caveat reworded to make clear the VID/PID are parameterized
        (`VendorID`/`ProductID`), not hardcoded, matching the Linux description and the actual
        `findShieldDevicePort.m` signature.
  - [x] Connection table and Test sequence step 1 updated to flag the observed PID
        `0x0100`/`0x0101` variability instead of stating `0x0101` as an unconditional fact.
  - [x] Step 4 updated with the interactive live-plot confirmation, closing the one gap left
        by headless-only real-hardware verification.
  - [x] Read through remaining sections (Protocol reference, Command format table, field index
        map, `comm_script.m` flow, Differences/porting notes, Prerequisites, No-hardware
        validation procedure, Commit sequence) against the shipped `.m` files — no further
        drift found; they match the implementation as built.
- **Definition of done**: README matches the shipped code.
  - **Done 2026-07-13.** All items above applied. The Commit sequence section's planned order
    (Steps 1→5, each its own commit, `test_connection.m` before `comm_script.m`) was followed
    exactly as originally planned — no deviation there to document.
- **Commit**: `docs(matlab): correct README per implementation findings`

## Next steps

Progress against the Work sequence above:

- [x] **Step 1** — `ShieldDevice.m` implemented, `checkcode`-clean, and verified against the
  no-hardware pty-loopback procedure (see Step 1's "Verified" note). Real-hardware
  confirmation still outstanding — folded into Step 3.
- [x] **Step 2** — `findShieldDevicePort.m` implemented and `checkcode`-clean. Verified against
  a real attached device's sysfs VID/PID (see Step 2's "Verified" note) — stronger than a
  pty-loopback pass, though the Windows path is still unverified. Flagged that the board's PID
  read `0x0100` at the time, not the `0x0101` this README and `comm_script.py` assume — by
  Step 3 the same physical board read `0x0101` again, so this looks like it can vary by mode/
  session rather than being a fixed hardware fact; treat it as a "check before assuming
  auto-detect will work," not a settled discrepancy.
- [x] **Step 3** — `test_connection.m` implemented, `checkcode`-clean, run twice against the
  real attached board: first unpowered (5/5 passed, validated protocol/discovery/parsing but
  not the measurement values), then **re-run powered at 30 Vdc** (5/5 passed, `V1` measured
  ≈4.96 V mean against a `REFERENCE LEG1 V1 5` command — confirms the analog measurement chain
  and closed-loop voltage regulation, not just the serial protocol). See Step 3's "Verified"/
  "Re-verified" notes. Both the protocol and the control loop are now confirmed against real
  hardware.
- [x] **Step 4** — `comm_script.m` implemented, `checkcode`-clean, and run headless
  (`EnablePlot=false`) against the real powered board for one full 200-frame cycle driving
  both legs 5→15→5 V: completed in 205.4 s with no errors, and the board was independently
  confirmed back in `IDLE` afterward. Fixed a dead-code bug found in `comm_script.py`'s
  plot-reset logic rather than reproducing it (see Step 4's "Verified" note and "Differences /
  porting notes"). **Live plot rendering visually confirmed** in an interactive MATLAB desktop
  session against the same powered board — see Step 4's final note. All four `.m` files now
  have real-hardware evidence.
- [x] **Step 5** — README reconciled against the shipped implementation: title/intro, the
  support-files table, the Windows caveat wording, the Connection table, and Test sequence
  step 1 were all updated (see Step 5's checklist for the full list); no further drift found
  on a full read-through. **The MATLAB port is complete**: implemented, statically clean, and
  verified end-to-end against real hardware — protocol (Steps 1–3), closed-loop regulation
  (Step 3 powered re-run), full command/ramp/measurement control-flow (Step 4 headless run),
  and live-plot rendering (Step 4 interactive confirmation).

If resuming cold: run `git log --oneline -- src/matlab/` to see which of the files above
already have commits, match that against the checkboxes here and in the corresponding Work
sequence block, and continue from the first unchecked item.
