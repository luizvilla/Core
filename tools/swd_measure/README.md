# SWD measurement tools (no scope)

PC side of `measure.h` / `measure.cpp`: talks to the firmware through an
ST-Link with OpenOCD (TCL port 6666) while it runs.

- `swd.py`: OpenOCD server, read / write of the `meas` block, requests,
  menu keys, buffer dump.
- `analyse.py`: frequencies from the counter wraps (halved in up-down mode),
  and logic-capture analysis (frequency, duty, overlap, dead time, PWMD /
  PWME phase).
- `test_spin.py <firmware.elf> <out.json>`: sequence for old17 (Spin app).
- `test_twist.py <firmware.elf> <out.json>`: sequence for old18 (per-leg
  frequency on a Twist).

Flash with `pio run -e STLink -t upload`, wait a few seconds after the
reset (boot) before the first request, then run e.g.
`python3 test_spin.py .pio/build/STLink/firmware.elf results.json`.
