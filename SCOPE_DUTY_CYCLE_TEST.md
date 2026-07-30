# Scope duty-cycle acceptance test

`src/tools/scope_duty_cycle_test.py` is a live-hardware script that exercises
the decimated software-scope feature (`/Debug/Scope`, `if00` data port —
see [src/README.md](src/README.md#software-scope)) against a real Twist
board. It runs three fixed-duty, open-loop power phases in sequence and
captures a scope buffer during each one:

1. **`leg1_only`** — Leg 1 enabled at a fixed duty cycle (default `0.3`),
   Leg 2 disabled.
2. **`leg2_only`** — Leg 2 enabled at the same fixed duty cycle, Leg 1
   disabled.
3. **`both_legs`** — both legs enabled at the same fixed duty cycle
   simultaneously.

No PID/tracking control is used (`wBuck`/`wBoost` stay `false`); each leg's
`wDutyCycle` is written directly. This deliberately exercises a different
code path than the closed-loop regulation test already recorded in
[automated_test.md](automated_test.md).

Each phase writes:

| File | Contents |
| --- | --- |
| `<phase>.csv` | Decoded capture: `time_s` plus all 8 channels, one row per sample, chronologically ordered (same format as `thingset_example.py --scope-csv`) |
| `<phase>.txt` | Human-readable report: requested config, pass/fail, pre- and post-capture `Measurements`, the `Debug/Scope` status object, and per-channel min/mean/max/peak-to-peak over the capture window |
| `<phase>.png` | Three stacked plots (voltages, currents, duty cycles) vs. time relative to the trigger, for quick visual inspection |

A `summary.txt` at the top of each run directory lists every phase's
pass/fail status and the parameters used.

## Preconditions

Same isolation and bench discipline as the rest of this project's live tests
(see [automated_test.md](automated_test.md)):

- High side energized at a known, operator-controlled voltage (nominally
  **30 V**) on an isolated, current-limited bench. This script never
  controls the external supply.
- **No external load** connected.
- Explicit, stable `/dev/serial/by-id/...` paths for the target board's
  `if02` (ThingSet shell) and `if00` (scope data) interfaces. There is no
  auto-detection, so a protected or unrelated board can never be opened by
  accident. Never open `if00` at 1200 baud (that requests MCUboot).
- Switching frequency is left at the compiled `200 kHz` default; the script
  never writes `Config/Frequency_Hz` (known firmware issue, waived in the
  prior acceptance run).
- If the target board is currently wedged from a previous run (see the
  "Dual-Serial Scope Acceptance Run" section of `automated_test.md`), it
  needs a physical reset before it will reflash or respond.

## Safety interlock

Because the applied duty cycle is open-loop, there is no PID feedback to
stop a runaway output on its own. The script polls `Measurements` every
`--poll-interval` seconds during the settle period and once more right after
the capture downloads, and immediately requests `POWER_OFF` plus a full
disconnect (`wEnable`/`wDriver`/`wCapa` = `false`) if any voltage magnitude
exceeds `--max-voltage` (default `35 V`) or any current magnitude exceeds
`--max-current` (default `3 A`). A tripped interlock stops the remaining
phases unless `--continue-on-error` is passed; every phase — successful,
failed, or interrupted — always ends with a shutdown request.

For reference, the Twist power stage is a synchronous buck, so with no load
the low-side voltages are expected to roughly track `duty × VHigh`: at the
default duty of `0.3` and `30 V` input, expect `V1Low_V`/`V2Low_V` around
`9 V`, comfortably under the default `35 V` interlock ceiling.

## Usage

```sh
python3 src/tools/scope_duty_cycle_test.py \
    --shell-port /dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_<serial>-if02 \
    --scope-port /dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_<serial>-if00 \
    --enable-power
```

`--enable-power` is mandatory and has no default — the script refuses to run
without it, as an explicit confirmation that the bench is prepared. Useful
options:

| Option | Default | Purpose |
| --- | --- | --- |
| `--duty` | `0.3` | Fixed duty cycle applied to every active leg |
| `--decimation` | `1` | Scope decimation (`1` → 102.4 ms window at full 100 us resolution) |
| `--pretrigger` | `0.2` | Scope pre-trigger ratio |
| `--settle-seconds` | `0.5` | Settle time after `POWER_ON` before arming/triggering the scope |
| `--max-voltage`, `--max-current` | `35 V`, `3 A` | Interlock limits |
| `--output-dir` | `scope_test_results` | Parent directory; each run gets its own UTC-timestamped subdirectory |
| `--continue-on-error` | off | Keep running remaining phases after a phase failure |

Run it from `src/tools` (or add that directory to `PYTHONPATH`) since it
imports `power_test_bench`, `scope_serial`, and `thingset_tools` as
siblings, same as `thingset_example.py`.

## Dependencies

`pyserial` and `matplotlib`, in addition to the existing `src/tools`
requirements. Neither is vendored; install them in whatever Python
environment runs the script.

## After the run

- Inspect `summary.txt` first for a per-phase pass/fail overview.
- Open a phase's `.png` for a quick look, its `.txt` for exact readback
  values and per-channel statistics, and its `.csv` if you need the raw
  samples (e.g. to re-plot or diff against another capture).
- The script always leaves the converter in `POWER_OFF` with both legs
  disabled and their drivers/capacitors disconnect-requested, whether or not
  every phase succeeded — matching the shutdown discipline used elsewhere in
  this project. Per the known limitation noted in `automated_test.md`,
  `wDriver` may still read back `true` after a disconnect request; a
  physical or MCU reset is the only way to fully clear it.
