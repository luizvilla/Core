# L2 · bench sessions on the board

Level L2 runs the flashed firmware on the real converter ([procedure §2](../../docs/testing/test-procedure.md#2-test-levels)). The steps, wiring and limits are in the bench sheet [inverter-session.md](../../docs/testing/bench/inverter-session.md). The **bench app** walks the tester through that sheet on a local web page. From the page the tester sends the menu keys, captures the Tektronix MSO54, answers the checks no instrument can make, and adds notes. The app logs the whole serial console, judges each criterion, and writes a campaign that the review page and the control center show like any other test. Nothing needs to be copied by hand.

`tests/run_all.sh` does not run L2: a bench session is run by hand, step by step.

## Requirements

| Item | Setup |
|---|---|
| `pyvisa`, `pyvisa-py`, `pyusb`, `pyserial` in `.venv` | `uv pip install --python .venv/bin/python pyvisa pyvisa-py pyusb pyserial` |
| USB access to the scope without root | `sudo cp tests/common/udev/99-tektronix-mso54.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug the scope |
| The board on USB with the `USB` firmware | The app opens `/dev/serial/by-id/*SPIN*` at 115200 baud. Close any `pio device monitor` first: only one program can hold the port |

## Run a session

```bash
.venv/bin/python tests/l2/app.py --new --note "400 V / 0.5 A, 100 W bulb, CH1/CH2 diff probes"
.venv/bin/python tests/l2/app.py          # later: reopen the open session (tests/out/bench-current)
```

The page opens at `http://localhost:8765`. The app listens on this computer only.

1. **Setup** (top right). Fill in the operator and the firmware on the board, and check what each scope channel carries. The default is the sheet's wiring: CH1 `V_inv`, CH2 `V_grid`, CH3 `I_line`, CH4 `relay_cmd`. Then press **Configure scope measurements**. The app creates the measurements the session needs, which you see on the scope: RMS, frequency and peak on every *ac* channel, plus the ones the criteria name, such as PHASE `V_inv`→`V_grid` with both sources set. Measurements already on the scope are reused. **Remove app measurements** deletes only the ones the app created.
2. **Pick a step** on the left, in sheet order. It shows the catalogue criteria and conditions, the step's menu keys, the oscilloscope panel, the criteria table, notes, and the sheet section.
3. **Start step** opens the telemetry window, and so does the first key you send from the step. Run the sheet's sequence with the step's key buttons. **x · Inverter OFF** is always at the top; the sheet's emergency stop still applies. Keys that close the relay or inject current ask for confirmation.
4. **Acquire** (or the space bar). If the scope is running, its statistics restart and gather for the window (2 s by default), and the criteria use the mean over that window. **Arm single shot** sets the scope to a single sequence and captures when it triggers, for relay closings and trips. Each capture goes into the point's history. The newest is the evidence unless you **pin** another. Tick *waveforms* to store the traces too (about 20 s); they are kept for checking, never judged.
5. **Answer** the operator checks (Pass/Fail plus what you saw, or text for INFO), and add **notes**.
6. **Plots** under the step:
   - **Oscilloscope waveforms** shows the stored MSO54 traces of the step's captures. Every **Arm single shot** stores them; a plain Acquire stores them only with *waveforms* ticked. The app sends a min/max envelope of the visible range, so a short spike stays visible when zoomed out, and zooming in fetches finer detail down to every sample.
   - **Firmware scope** shows the dumps from key `r` (14 channels, 256 samples, time 0 at the trigger). The firmware prints the trigger sample and the rate before each dump (`scope: 256 samples, trigger at sample 26, 2000 Hz`, trigger at 10 % since 2026-11-13); for older dumps the rate is guessed from the console and the trigger is sample 129. The rate selector corrects a wrong guess.
   - Both plots: tick the channels to show (groups and All/None buttons), choose stacked or overlaid axes, drag to zoom, wheel to zoom at the cursor, shift+drag to pan, double-click to reset; the hover box gives every visible value at that instant.
7. **Finish step** closes the telemetry window. The verdicts, `record.md`, the review page and the control center update after every change. The header links to them.

## How a criterion is judged

[`criteria.json`](criteria.json) defines the session: the default channels, the standard measurements per kind of channel, the menu keys, and one entry per L2 test. The title, purpose and conditions stay in [`../catalogue.json`](../catalogue.json). The limits are copied unchanged from the bench sheet.

A criterion has one or more parts, and all of them must pass:

| Part | Example | Verdict |
|---|---|---|
| `value`: expression over the scope's own measurements | `100*(RMS(V_inv)-RMS(V_grid))/RMS(V_grid)`, `lo` −3, `hi` 3 | PASS / FAIL; INFO without limits (the sheet states no number) |
| `value` with `points` and `agg` | `1000*DELAY_RISE_FALL(relay_cmd,relay_contact)` over `close1…3`, `agg` `max` | the worst over the points (`max`, `min`, `maxabs`); BLOCKED until every point is captured |
| `telemetry`: conditions on the console over the step window | `{"field": "rly", "all": "00"}`, `{"field": "inv", "holds": "8"}`, `{"field": "inv", "reaches": "3"}`, `{"console_lacks": "trip"}`, `{"time_from_key": "c", "field": "inv", "value": "8"}` | PASS / FAIL; INFO for timings |
| `operator` | `true`: the tester's Pass/Fail; `"info"`: text | as answered |

`NAME(sig)` or `NAME(sig1,sig2)` is the scope measurement of type `NAME` (as `MEASUrement:MEASx:TYPe?` returns it) on the channels the session maps those signals to. `DELAY_RISE_FALL(a,b)` (any `RISE`/`FALL` pair) is the scope DELAY from that edge of `a` to the next edge of `b`; Configure sets `FROMedge`, `TOEdge` and a forward search. `abs`, `max`, `min` are plain functions: `max(MAXIMUM(I_line),-MINIMUM(I_line))` is the peak of either sign. A test's `channels` rewires channels for that step only (step 1b puts the relay contact on CH3), and the page says so under the oscilloscope. A telemetry condition may carry `after_key` (from the last time that key was sent in the step) or `before_key` (until the first time after that), so a step with a protection test can judge "no fault" before `z` and "trip" after it. A criterion is `BLOCKED`, with the reason, when:
- its point is not captured yet;
- its measurement is missing or has no value (*Low amplitude*);
- the step was never started;
- no telemetry line fell in the window;
- a required operator answer is missing.

Telemetry conditions read the short telemetry line of `src/main.cpp` (`st … inv … rly …`, every 500 ms). Telemetry fields: `st`, `inv`, `rly`, `VAC`, `IAC`, `Vbus`, `Id`, `Iq`, `ok`, `flt`, `off`, `load`, `lost`. A line garbled by printk is logged but never judged.

| Test | Points | Scope | Telemetry | Operator |
|---|---|---|---|---|
| L2-INV-10 pre-checks | — | — | .b menu printed, st 0, inv 0, rly 00 | .a, .c, .d |
| L2-INV-00 open loop | 100V … 400V | .a RMS(V_inv) 218–242 V and .b FREQUENCY(V_inv) 49.9–50.1 Hz at 400 V; .e RMS at 100–340 V (INFO) | .c rly 00, flt 0, no trip message | .d (INFO), .f |
| L2-INV-01 synchronized, relay open | main | .a RMS within ±3 %; .b PHASE within ±3°; .c frequency difference (INFO) | .d rly 00 and inv 8 held; .e time from c to state 8 (INFO) | .d bulb dark, .e (INFO) |
| L2-INV-04 dry run, relay contacts unwired (CH3 = contact) | close1–3, release1–3, 1s, 10s | .b DELAY CH4 rise → CH3 fall ≤ 15 ms, worst of 3; .c DELAY CH4 fall → CH3 rise ≤ 15 ms, worst of 3; .d RMS ±3 % and PHASE ±3° at 10 s (1 s INFO) | .a inv reaches 3, rly reaches 11, flt 0, before `z`; .f trip message and inv 7 after `z` | .a sequence, .b bounce, .c, .e (INFO), .f |
| L2-INV-02 relay closing | close, connected, open | .a peak CH3 < 3 A; .c RMS CH3 < 0.05 A | .a, .d no trip message; .d final rly 00, inv 0 | .b settling, .c bulb, .d, .e/.f (INFO) |
| L2-INV-07 step 2b (`G`) | close, connected, open | as L2-INV-02 | as L2-INV-02 | as L2-INV-02, .g 20 ms ±2 ms (firmware capture), .h (INFO) |
| L2-INV-08 step 2b, SOGI feedforward (`F`) | step1-measured, step1-sogi, close, connected, open | as L2-INV-07; .i RMS and PHASE in step 1 with each source (INFO) | as L2-INV-07, plus `ff 1` from `G` (.ff) | as L2-INV-07 |
| L2-INV-03 inject | 0.02A, 0.05A, 0.1A | .a CH3 RMS·√2 within ±5 mA / ±10 % of Id_ref per point; .b PHASE CH3 → CH2 within ±10°, worst point | .c VAC 207–253 V and ok 1 after `n` | .d, .e (INFO) |

L2-INV-05 and L2-INV-06 (steps 1c, 1d) were retired by [ADR 0007](../../docs/adr/0007-apf-quadrature-and-averaged-ac-reads.md): their keys (`+`, `-`, `a`) are gone from the firmware and from `criteria.json`. The relay timings (closing, release) are scope DELAY measurements; the bounce, the settling and the 20 ms state change of step 2b stay operator checks read on the captures, as no single scope measurement gives them. The PHASE criterion of L2-INV-03 assumes the current probe arrow points from the inverter to the grid.

## Command line

[`bench.py`](bench.py) does the same from a terminal, for scripts and agents. It uses the same session files and is not needed by the tester:

```bash
.venv/bin/python tests/l2/bench.py status
.venv/bin/python tests/l2/bench.py capture L2-INV-00 --point 400V
.venv/bin/python tests/l2/bench.py check L2-INV-10 --check .a=pass --check ".c=fail:limit at 0.3 A"
.venv/bin/python tests/l2/bench.py step L2-INV-01 finish
```

## Files

| File | Role |
|---|---|
| `app.py` | Local server (127.0.0.1) and JSON API; owns the serial port and the scope |
| `web/index.html` | The page (no external dependencies, works offline) |
| `session.py` | Session model: campaign files, measurement plan, judging, bench-sheet text |
| `console.py` | Serial reader: `console.txt`, telemetry parsing, keys, firmware scope dumps (key `r`, decoded to CSV) |
| `bench.py` | Command line |
| `criteria.json` | Channels, measurements, keys and criteria per test |

A session campaign, `tests/out/<date>_<sha>_<time>_bench/`, holds:

```
bench_session.json       operator, firmware, note, channel map, measurements the app created
console.txt              every console line and key, time-stamped
firmware_scope/          firmware scope dumps (.txt as received, .csv decoded)
l2_inv01/bench_manifest.json   step window, captures per point (pinned one), operator answers, notes
l2_inv01/captures/<time>_<point>/   screen.png, capture.json, summary.md, waveforms.npz
l2_inv01.log             result lines of the test, rewritten on every change
record.md, report.html
```

The review page renders each `l2_*` folder with the four parts (`tests/common/review_report.py`, kind *bench*):
1. Catalogue criteria and conditions, step window, capture points, channel map, notes.
2. Bench snapshot: firmware, operator, scope setup per point.
3. Results.
4. The evidence screenshot per point with its measurement table, the waveforms if stored, one interactive figure per firmware scope dump (panels by quantity, time 0 at the trigger, click a legend entry to hide a signal; the sample rate recorded with the dump, else guessed from the console), and the console of the step window.

> **Note:** the firmware does not print its version yet, so the session records the repository HEAD unless the tester sets the firmware in Setup.

## History

- 2026-11-13: L2-INV-08 (step 2b with the SOGI feedforward, key `F`); telemetry field `ff` parsed; `v` optional in step 2b.
- 2026-11-13: plots of the oscilloscope waveforms (envelope per zoom) and of the firmware scope dumps; single shots always store the waveforms. Fix: firmware scope CSVs were not rotated (the index line is `## n`), so the trigger was not at sample 128; the app re-decodes old dumps from their `.txt`.
- 2026-11-13 (ADR 0007): steps 1c/1d retired (L2-INV-06 criteria, keys `a`, `+`, `-` removed; stage 5 dropped from the stage names); key `G` relabelled, keys `i` and `v` added. L2-INV-04, 02, 07 and 03 defined: keys, capture points, scope criteria (DELAY with edges, peak, RMS, PHASE), telemetry and operator checks. New in the engine: per-step channels, DELAY edges, worst over points, `reaches`, `after_key`/`before_key`.
- 2026-11-13: bench app: page with menu keys, console, scope acquisition with history and pinning, operator answers, telemetry criteria; scope measurements created per session. Same day: first version (`bench.py`, scope capture per step).
