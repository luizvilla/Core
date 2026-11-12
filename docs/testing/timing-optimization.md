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
| 1 | `load` bookkeeping: cycle counter read directly, raw cycles in the task, µs conversion in the telemetry thread | `src/main.cpp` | Measured alone: **−6.97 µs** (branch `perf/timing-1-bookkeeping`, `527e36d`) |
| 2 | Scope: 10 kHz around the relay only on request (2 kHz by default) | `src/main.cpp` | Measured alone: **−3.7 µs in g and G, −2.9 µs in n**, 0 elsewhere (branch `perf/timing-2-scope`, `c96d94f`) |
| 3 | PWM and relay writes: probe each call, then send the state commands only on change | `src/main.cpp` (and the shield API if needed) | Planned |
| 4 | Modulos (Jean Alinei's feedback): `fmodf` of θ every tick → conditional subtraction; integer `%` of the sync window → conditional wrap | `single_phase_inverter` fork | Planned |
| All | 1–4 together: final verdict | | Planned |

Then: the ADC path (data dispatch 9.4 µs + sensor reads 8.2 µs per period), with Jean.

## Results

(one section per change, filled as it is measured)

### 1 · Load bookkeeping (`527e36d`, campaign `2026-11-12_527e36d_151613_MrgUHA`)

The task called `k_cycle_get_32()` five times per tick and converted five intervals with `k_cyc_to_us_floor32()` (a 64-bit division). It now reads the cycle counter (`DWT->CYCCNT`, one load, enabled in `setup_routine`) and keeps cycle counts; the telemetry thread divides by 170. Same telemetry fields, same lost-tick rule.

| Stage | Interrupt / period (mean) | Task tick worst | Lost | Writes (mean) | Scope (mean) |
|---|---|---|---|---|---|
| OFF_IDLE | 56.4 → 50.1 (−6.3) | 52.0 → 45.0 | 0 → 0 | 2.57 → 1.18 | 2.86 → 1.16 |
| OFF_BUSREG | 62.8 → 55.9 (−6.9) | 58.4 → 51.5 | 0 → 0 | 7.21 → 5.82 | 3.75 → 1.47 |
| OFF_MPPT | 63.6 → 56.7 (−6.9) | 58.9 → 51.4 | 0 → 0 | 6.95 → 5.59 | 3.77 → 1.47 |
| SETTLE_c | 66.9 → 60.0 (−6.9) | 62.4 → 55.0 | 0 → 0 | 10.65 → 9.29 | 3.47 → 1.15 |
| CONNECT_g | 71.0 → 63.6 (−7.4) | 66.4 → 58.7 | 0 → 0 | 10.59 → 9.24 | 7.13 → 4.30 |
| INJECT_n | 71.7 → 64.5 (−7.2) | 63.8 → 58.9 | 0 → 0 | 10.71 → 9.35 | 6.21 → 3.59 |
| VAC_FOLLOW_a | 69.2 → 62.3 (−6.9) | 64.2 → 56.8 | 0 → 0 | 10.78 → 9.41 | 3.77 → 1.45 |
| FOLLOW_CONN_G | 71.8 → 64.4 (−7.4) | 68.7 → 61.3 | 0 → 0 | 10.60 → 9.24 | 7.19 → 4.36 |
| OPEN_LOOP_o | 70.1 → 63.2 (−6.9) | 65.3 → 57.8 | 0 → 0 | 10.77 → 9.41 | 3.31 → 1.00 |

Each tick part loses about 1.4 µs (its closing `k_cycle_get_32()` and conversion), the sensor reads 0.04 µs; the controlStep sections do not change. More than the 4 µs estimated from the grey segment, because part of the bookkeeping sat inside the measured parts. L0-TIM-01/02/03 all pass; background tasks served.

### 2 · Scope at 2 kHz by default (`c96d94f`, campaign `2026-11-12_c96d94f_151814_LKQXmD`)

`scope_fast_connect` now starts false: the scope samples every 5th tick in every stage; key `v` still switches CONNECT, INJECT and FOLLOW_CONNECT to every tick.

| Stage | Interrupt / period (mean) | Task tick worst | Lost | Scope (mean) |
|---|---|---|---|---|
| CONNECT_g | 71.0 → 67.3 (−3.7) | 66.4 → 62.9 | 0 → 0 | 7.13 → 3.48 |
| INJECT_n | 71.7 → 68.8 (−2.9) | 63.8 → 64.1 | 0 → 0 | 6.21 → 3.30 |
| FOLLOW_CONN_G | 71.8 → 68.1 (−3.7) | 68.7 → 65.7 | 0 → 0 | 7.19 → 3.51 |
| other stages | unchanged (within 0.05 µs) | unchanged | 0 → 0 | unchanged |

The average load drops, not the worst tick: every 5th tick still takes the full scope sample (about 7 µs), and in INJECT that sampled tick is the worst one (64.1 µs). Mean over the 9 stages: −1.13 µs.
