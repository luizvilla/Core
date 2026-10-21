# Per-leg switching frequency: Ownverter test

Tests the per-leg switching frequency (`leg-frequency` in the device tree,
Core branch `feature/hrtim-per-unit-frequency`) through the Power API on an
**Ownverter 1.1.0**.

> **Safety.** No DC supply on the Ownverter power input during this test.
> The app starts the PWM signals with the gate drivers **disabled**; they are
> checked on the Spin pins. `e` enables the gate drivers (the power stage
> then switches): only use it without a DC bus, if at all.

## Setup

This is a **test branch** for `feature/hrtim-per-unit-frequency`, not to be
merged: it is the feature branch plus this app in `src/`.

1. `git checkout test/hrtim-freq-ownverter` in the Core repository.
2. Clean build (`app.overlay` is only picked up at CMake configuration), upload (`USB` environment), open the serial monitor,
   press `h`.

Do not use a project folder whose name contains `app` (e.g. `app-test`):
PlatformIO then skips the OwnTech modules and the link fails.

## What the app does

All legs are center-aligned (Ownverter default), buck convention, 100 ns
dead time (Ownverter default). `app.overlay` lowers `min-frequency` to 50 kHz
and sets:

| Leg | Timer | Pins (high / low) | Frequency | Duty cycle (high side) |
|---|---|---|---|---|
| LEG1 | TIMA | PA8 / PA9 (Spin 12 / 14) | common `default-frequency`, 200 kHz | 0.50 |
| LEG2 | TIMC | PB12 / PB13 (Spin 2 / 4) | `leg-frequency` 100 kHz | 0.25 |
| LEG3 | TIME | PC8 / PC9 (Spin 10 / 11) | `leg-frequency` 50 kHz | 0.30 |

Serial keys:

| Key | Action |
|---|---|
| `h` | help |
| `p` | print frequency, period and duty cycle of each leg |
| `o` | PWM outputs on / off (gate drivers untouched) |
| `u` / `j` | common frequency +/- 10 kHz: `shield.power.setFrequency(f)`, only LEG1 must change |
| `d` / `c` | LEG2 frequency +/- 10 kHz: `shield.power.setFrequency(LEG2, f)` |
| `f` / `v` | LEG3 frequency +/- 10 kHz: `shield.power.setFrequency(LEG3, f)` |
| `s` | `setPhaseShift(LEG3, 90)`: must have no effect (own-frequency leg) |
| `e` / `q` | gate drivers on / off (see Safety) |

## Checks (scope on the Spin pins)

1. **Frequencies.** LEG1 200 kHz (5 us), LEG2 100 kHz (10 us), LEG3 50 kHz
   (20 us). `p` prints the same values.
2. **No truncation.** LEG2 and LEG3 show clean periods. Without the fix, a
   unit at another frequency than the master was still reset every master
   period (5 us) and its waveform was cut.
3. **Duty cycles.** High-side pulse widths: LEG1 2.5 us, LEG2 2.5 us, LEG3
   6 us (+/- dead time).
4. **Complementary outputs and dead time.** On each leg, low = complement of
   high, about 100 ns where both are low, on every edge.
5. **Common frequency change.** `u` / `j`: LEG1 changes (210, 190 kHz...),
   LEG2 and LEG3 do not move. LEG1 duty cycle stays 0.50.
6. **Per-leg frequency change.** `d` / `c` on LEG2, `f` / `v` on LEG3: only that
   leg changes, its duty cycle stays the same. Below 50 kHz the change is
   refused (`... refused`). `setFrequency(LEG1, f)` would be refused too (LEG1
   uses the common frequency): use `u` / `j` for it.
7. **Phase shift refused.** `s`: LEG3 waveform unchanged.
8. **Outputs off / on.** `o` stops the three legs (both outputs low), `o`
   again restarts them with the same settings.
9. **Regression.** Remove `app.overlay` from `src/`, clean build: all legs
   back at 200 kHz, `p` shows 200 kHz everywhere (no `leg-frequency`, behaviour
   of the Core before the branch).

Please report: frequencies measured in 1, any truncation in 2, the dead time
measured in 4, anything in 5 / 6 / 7 that moved when it should not have, and
the result of 9.
