# Preliminary test results: per-leg frequency (Ownverter app)

Measured on 2026-10-21. Core `feature/hrtim-per-unit-frequency` (520e528).

**Not run on an Ownverter.** The same code path (Power API, `leg-frequency`)
and the same timers (A, C, E) were tested on a Spin + Twist 1.4.2 with the
power side off, with a Twist build of this app (`old18`):
- LEG1 / TIMA at the common 200 kHz;
- LEG2 / TIMC at 100 kHz;
- LEG3 / TIME at 50 kHz, a leg added in the overlay;
- Twist 100 ns dead time.

**No scope**: measured over SWD (ST-Link):
- a logic capture of the pins at 100 ns per sample;
- HRTIM registers read while running;
- counters timed against the CPU cycle counter.

Gate drivers off. **No failure found.**

## Initial state

| Leg | Frequency | Duty high (expected) | Both high | Dead time | Reset by the master |
|---|---|---|---|---|---|
| LEG1 | 199 990 Hz | 0.48 (0.48) | never | 100 ns | yes (common frequency) |
| LEG2 | 99 995 Hz | 0.24 (0.24) | never | 100 ns | no |
| LEG3 | 49 997 Hz | 0.295 (0.295) | never | 100 ns | no |

## Checklist (README)

| # | Check | Result |
|---|---|---|
| 1 | Frequencies 200 / 100 / 50 kHz | ✓ |
| 2 | No truncation | ✓ counters reach their full period (LEG3: 27 184 / 27 200) |
| 3 | Duty cycles | ✓ (table above, duty − dead time) |
| 4 | Complementary outputs, dead time | ✓ never both high, 100 ns |
| 5 | `u` / `j`: common frequency | ✓ LEG1 210 / 190 / 200 kHz; LEG2, LEG3 unchanged |
| 6 | `d` / `c`, `f` / `v`: per-leg frequency | ✓ LEG2 110 kHz (110 001 Hz, duty 0.239) and back; LEG3 60 kHz and back; LEG3 40 kHz refused; other legs unchanged |
| 7 | `s`: phase shift on LEG3 | ✓ ignored (registers and frequency unchanged) |
| 8 | `o`: outputs off / on | ✓ |
| 9 | Regression without `leg-frequency` | ✓ all legs at 200 kHz, reset by the master as before |

Not covered: the Ownverter board itself (driver pins, its overlay values), the
gate drivers (`e` / `q`), and precise dead time and edge shapes (needs a scope).
