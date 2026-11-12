# Critical task timing: optimizations, one change at a time

Tracks the changes that bring the critical task's time down, measured with the on-target timing tests (L0-TIM-01/02/03, [ADR 0005](../adr/0005-on-target-timing-tests.md), [tests/l0/timing/README.md](../../tests/l0/timing/README.md)) on the bare Spin. Each change goes on its own branch from the same base and is measured alone; then all are merged and measured together. Production changes also get a [worklog](worklog.md) entry.

## Method

| Item | Rule |
|---|---|
| Base | `a528915` (4 flash wait states, TIM-01/02/03 at TIM-03's detail), campaign `2026-11-12_6d00a25_150416_YU7So7` |
| One change at a time | Branch `perf/timing-<n>-<name>` from the base; one TIMING run (`tests/l0/timing/run.sh --run`) |
| Comparison | `python3 tests/l0/timing/compare.py <base campaign> <change campaign> --rows <parts>` |
| Main figure | **Interrupt time per period** (mean, L0-TIM-03): the task API's safety check and ADC data dispatch plus the critical task tick, without the test harness |
| Noise | Two runs of the same code: means within 0.05 µs, worst tick within 0.1 µs, worst gap within about ±5 µs |
| Behaviour | Host L0 campaign: no new failure. The USB firmware builds and has no timing symbol |

## Base (a528915)

| Stage | Interrupt / period (mean) | Task tick worst | Lost |
|---|---|---|---|
| OFF_IDLE | 56.4 µs | 52.0 µs | 0 |
| OFF_BUSREG | 62.8 µs | 58.4 µs | 0 |
| OFF_MPPT | 63.6 µs | 58.9 µs | 0 |
| SETTLE_c | 66.9 µs | 62.4 µs | 0 |
| CONNECT_g | 71.0 µs | 66.4 µs | 0 |
| INJECT_n | 71.7 µs | 63.8 µs | 0 |
| VAC_FOLLOW_a | 69.2 µs | 64.2 µs | 0 |
| FOLLOW_CONN_G | 71.8 µs | 68.7 µs | 0 |
| OPEN_LOOP_o | 70.1 µs | 65.3 µs | 0 |

## Changes

| # | Change | Where | Status |
|---|---|---|---|
| 1 | `load` bookkeeping: cycle counter read directly, raw cycles in the task, µs conversion in the telemetry thread | `src/main.cpp` | Planned |
| 2 | Scope: 10 kHz around the relay only on request (2 kHz by default) | `src/main.cpp` | Planned |
| 3 | PWM and relay writes: probe each call, then send the state commands only on change | `src/main.cpp` (and the shield API if needed) | Planned |
| 4 | Modulos (Jean Alinei's feedback): `fmodf` of θ every tick → conditional subtraction; integer `%` of the sync window → conditional wrap | `single_phase_inverter` fork | Planned |
| All | 1–4 together: final verdict | | Planned |

Then: the ADC path (data dispatch 9.4 µs + sensor reads 8.2 µs per period), with Jean.

## Results

(one section per change, filled as it is measured)
