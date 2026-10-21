# Preliminary test results: Spin app (per-unit HRTIM frequency)

Measured on 2026-10-21 on a Spin (STM32G474) mounted on a Twist 1.4.2 with the
power side off. Core `feature/hrtim-per-unit-frequency` (520e528).

**No scope**: measured over SWD (ST-Link) with a measurement build of this app
(`old17`):
- a logic capture of the pins at 100 ns per sample;
- HRTIM registers read while running;
- counters timed against the CPU cycle counter.

Dead times are therefore known to ±100 ns. **No failure found.**

## Initial state

| Unit | Frequency | Duty high (expected) | Duty low (expected) | Both high | Dead time |
|---|---|---|---|---|---|
| PWMA | 199 988 Hz | 0.46 (0.46) | 0.46 (0.46) | never | 200 ns |
| PWMC | 99 994 Hz | 0.23 (0.23) | 0.73 (0.73) | never | 200 ns |
| PWMD | 49 997 Hz | 0.29 (0.29) | 0.69 (0.69) | never | 200 ns |
| PWME | 49 997 Hz | 0.69 (0.69) | 0.29 (0.29) | never | 200 ns |

The high side loses one dead time per period, so the expected duty is d − t_dead × f.

## Checklist (README)

| # | Check | Result |
|---|---|---|
| 1 | Frequencies 200 / 100 / 50 / 50 kHz | ✓ (table above) |
| 2 | No truncation of PWMD / PWME | ✓ counters reach PER − 16 (27 184 / 27 200), clean 20 µs periods |
| 3 | Duty cycles | ✓ |
| 4 | Complementary outputs, dead time | ✓ never both high, 200 ns on every edge |
| 5 | PWMD / PWME in phase | ✓ pulse centres coincide (0 ns); after `r`: 100 ns (1 sample) |
| 6 | `u` / `j`: common frequency | ✓ PWMA 210 / 190 / 200 kHz, duty kept; PWMC, PWMD, PWME unchanged |
| 7 | `d` / `c`: PWMD frequency | ✓ PWMD 60 kHz (59 999 Hz, duty 0.288), back to 50 kHz; 40 kHz refused; others unchanged |
| 8 | `o`: outputs off / on | ✓ all pins low, then restored with the same settings |

Not covered: precise dead time and edge shapes (needs a scope).
