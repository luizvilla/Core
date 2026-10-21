# Per-unit HRTIM frequency: measurement build (no scope)

Test branch for `feature/hrtim-per-unit-frequency`, **not to be merged**.
Same app as the scope test branch (`test/hrtim-freq-ownverter`), plus a measurement module
(`measure.h` / `measure.cpp`) driven over SWD with an ST-Link. Use it when no
oscilloscope is available.

## Setup

1. `git checkout test/hrtim-freq-measure-twist` in the Core repository (project folder name must
   not contain `app`, otherwise PlatformIO skips the OwnTech modules).
2. Clean build and flash through the ST-Link: `pio run -e STLink -t upload`.
3. Wait a few seconds after the reset, then run the sequence:
   `python3 tools/swd_measure/test_twist.py .pio/build/STLink/firmware.elf results.json`

The script starts OpenOCD (PlatformIO `tool-openocd`), sends the menu keys
through the debugger (the USB console is not needed), and writes registers,
counter-based frequencies and logic-capture analysis (frequency, duty cycle,
overlap, dead time) for each step to `results.json`. See
`tools/swd_measure/README.md`.

Board: Spin on a **Twist 1.4.2, power side off**. This is the Ownverter app
on the Twist (same timers A, C, E; LEG3 added on TIME in `app.overlay`):
it tests the Power API / `leg-frequency` path without an Ownverter.

## How it measures

- **Registers**: HRTIM registers copied while running.
- **Frequency**: counter wraps over 100 ms against the CPU cycle counter,
  halved in up-down mode (CNTxR reads as a rising sawtooth twice per PWM
  period, RM0440 27.3.6).
- **Logic capture**: PA8/PA9, PB12-PB15, PC8/PC9 sampled every 100 ns
  (4.9 ms), so dead times are known to +/- 100 ns.

`preliminary_test_results.md`: results obtained with this branch on
2026-10-21.
