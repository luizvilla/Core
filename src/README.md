# Per-unit HRTIM frequency: Spin test

Tests the per-timing-unit frequency of the HRTIM (Core branch
`feature/hrtim-per-unit-frequency`) on a **Spin alone**, without any power
shield. Every check is done with a scope on the Spin pins.

## Setup

This is a **test branch** for `feature/hrtim-per-unit-frequency`, not to be
merged: it is the feature branch plus this app in `src/`.

1. `git checkout test/hrtim-freq-spin` in the Core repository.
2. Clean build, upload (`USB` environment), open the serial monitor,
   press `h`.

Do not use a project folder whose name contains `app` (e.g. `app-test`):
PlatformIO then skips the OwnTech modules and the link fails.

## What the app does

All units are center-aligned, with a 200 ns dead time. The common frequency
is 200 kHz; the minimum frequency is 50 kHz (this sets the HRTIM clock
prescaler shared by all the units).

| Unit | Pins (high / low) | Frequency | Duty cycle (high side) |
|---|---|---|---|
| PWMA | PA8 / PA9 (Spin 12 / 14) | common, 200 kHz | 0.50 |
| PWMC | PB12 / PB13 (Spin 2 / 4) | own, 100 kHz | 0.25 |
| PWMD | PB14 / PB15 (Spin 5 / 6) | own, 50 kHz | 0.30 |
| PWME | PC8 / PC9 (Spin 10 / 11) | own, 50 kHz | 0.70 |

PWMD and PWME are restarted together after their init, so they stay in
phase.

Serial keys:

| Key | Action |
|---|---|
| `h` | help |
| `p` | print frequency, period and duty cycle of each unit |
| `o` | outputs on / off |
| `u` / `j` | common frequency +/- 10 kHz (only PWMA must change) |
| `d` / `c` | PWMD frequency +/- 10 kHz (only PWMD must change) |
| `r` | restart PWMD and PWME together |

## Checks (scope)

1. **Frequencies.** PWMA 200 kHz (5 us), PWMC 100 kHz (10 us), PWMD and PWME
   50 kHz (20 us). `p` prints the same values.
2. **No truncation.** PWMD and PWME are clean 20 us periods. Before the fix,
   timer D/E were reset by the master every 5 us and their waveform was cut.
3. **Duty cycles.** High-side pulse widths: PWMA 2.5 us, PWMC 2.5 us, PWMD
   6 us, PWME 14 us (+/- dead time).
4. **Complementary outputs and dead time.** On each unit, low = complement
   of high, with about 200 ns where both are low, on every edge.
5. **PWMD / PWME in phase.** The centres of their high pulses coincide
   (center-aligned, same carrier). Press `r`: still in phase.
6. **Common frequency change.** `u` / `j`: PWMA changes (190, 180 kHz...),
   PWMC, PWMD and PWME do not move. Its duty cycle stays 0.50.
7. **Own frequency change.** `d` / `c`: PWMD changes (60, 40 kHz...), its duty
   cycle stays 0.30; PWMA, PWMC and PWME do not move. Below 50 kHz the change
   is refused (`PWMD: ... refused`). After `d` / `c`, PWMD and PWME are no
   longer at the same frequency; set PWMD back to 50 kHz and press `r` to
   align them again.
8. **Outputs off / on.** `o` stops all the outputs (both low), `o` again
   restarts them with the same settings.

Please report: frequencies measured in 1, any truncation in 2, the dead
time measured in 4, the phase between PWMD and PWME in 5, and anything in
6 / 7 that moved when it should not have.
