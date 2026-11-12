# Critical task timing: optimizations, one change at a time

Tracks the changes that bring the critical task's time down, measured with the on-target timing tests (L0-TIM-01/02/03, [ADR 0005](../adr/0005-on-target-timing-tests.md), [tests/l0/timing/README.md](../../tests/l0/timing/README.md)) on the bare Spin. Each change goes on its own branch from the same base and is measured alone; then all are merged and measured together. Production changes also get a [worklog](worklog.md) entry.

## Method

| Item | Rule |
|---|---|
| Base | `a528915` (4 flash wait states, TIM-01/02/03 at TIM-03's detail), campaign `2026-11-12_6d00a25_150416_YU7So7`; from change 3 on, `23afd86` (same code plus TIMING-only probes inside the writes, +0.34 µs in the TIMING build), campaign `2026-11-12_23afd86_152108_EewYf2` |
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
| 3 | PWM and relay writes: probe each call, then send the state commands only on change | `src/main.cpp` | Measured alone: **−2.1 to −2.3 µs with the boost running**, 0 in OFF_IDLE (branch `perf/timing-3-writes`, `fb07a70`); bridge duty API: proposal 3b below |
| 4 | Modulos (Jean Alinei's feedback): `fmodf` of θ every tick → conditional subtraction; integer `%` of the sync window → conditional wrap | `single_phase_inverter` fork `ac3dbc6` | Measured alone: **−0.4 to −0.5 µs** in every stage (branch `perf/timing-4-modulo`, `c664d8d`) |
| All | 1–4 together: final verdict | merge `5fcf14b` | **−10.15 µs** mean over the stages (−6.3 to −13.2 µs per stage), every test passes |

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

### 3 · PWM and relay writes (`fb07a70`, campaign `2026-11-12_fb07a70_152338_MMmrUV`, against `23afd86`)

First the writes were split by call (TIMING-only probes, `23afd86`). Mean per tick in CONNECT `g`:

| Call | µs | Every tick needed? |
|---|---|---|
| Boost duty `setDutyCycle(LEG1_LOW)` | 1.82 | yes (value changes) |
| Boost enable `start(LEG1_LOW)` | 2.48 | no: the leg was already on |
| Burst `setBurstMode` | 0.61 | only when skip/period change |
| Bridge duty `setInverterDutyCycle` | 3.38–3.50 | yes (value changes); the cost is the API's validation |
| Bridge enables | 0.15 | already on change |
| PWM acknowledgement | 0.34 | — |
| Relay | 0.37–0.40 | already on change |

Change: boost leg 1 `start`/`stop` and `setBurstMode` are sent when the command changes. **Behaviour change:** a stop from the safety API (`safety_action()` stops ALL legs) is no longer undone by the next tick for boost leg 1; it holds until the next mode change, as it already did for the bridge.

| Stage | Interrupt / period (mean) | Task tick worst | Lost | Boost enable (mean) | Writes (mean) |
|---|---|---|---|---|---|
| OFF_IDLE | 56.7 → 56.7 (+0.1) | 52.1 → 52.3 | 0 → 0 | 0.02 → 0.02 | 2.76 → 2.85 |
| OFF_BUSREG | 63.2 → 60.9 (−2.2) | 58.8 → 57.7 | 0 → 0 | 2.48 → 0.14 | 7.44 → 5.27 |
| OFF_MPPT | 64.0 → 61.8 (−2.2) | 59.2 → 57.0 | 0 → 0 | 2.48 → 0.14 | 7.26 → 5.22 |
| SETTLE_c | 67.2 → 65.0 (−2.2) | 62.7 → 60.5 | 0 → 0 | 2.48 → 0.14 | 10.83 → 8.73 |
| CONNECT_g | 71.3 → 69.0 (−2.3) | 66.6 → 64.3 | 0 → 0 | 2.48 → 0.14 | 10.81 → 8.64 |
| INJECT_n | 72.1 → 69.8 (−2.3) | 64.2 → 61.9 | 0 → 0 | 2.48 → 0.14 | 10.93 → 8.76 |
| VAC_FOLLOW_a | 69.5 → 67.3 (−2.2) | 64.4 → 62.3 | 0 → 0 | 2.48 → 0.14 | 10.95 → 8.84 |
| FOLLOW_CONN_G | 72.2 → 69.9 (−2.3) | 69.0 → 66.8 | 0 → 0 | 2.48 → 0.14 | 10.81 → 8.65 |
| OPEN_LOOP_o | 70.3 → 68.2 (−2.1) | 65.4 → 63.4 | 0 → 0 | 2.49 → 0.14 | 10.97 → 8.85 |

On the bench the boost is idle, so this change shows only when the boost runs. The burst check saves 0.13 µs (`setBurstMode` was cheap).

**Proposal 3b (not done, shield API):** `setInverterDutyCycle()` re-validates the whole pair on every call (periods, prescalers, frequencies, modulation, mode, phase shifts, switch conventions) and recomputes the duty limits before two register writes. Caching the validated pair and its raw limits when the configuration changes (`initInverter`, duty min/max, frequency, convention) would leave clamp, convert and write: most of 3.4 µs per tick in every bridge stage. It changes the OwnTech shield API (cache invalidation), so it needs Jean's and Clément's agreement.

### 4 · Modulos (library `ac3dbc6`, Core2 `c664d8d`, campaign `2026-11-12_c664d8d_152708_cxhv2o`, against `23afd86`)

Audit of the critical path: the inverter library wrapped θ with `fmodf` every tick and indexed the sync window with two integer `%` per tick; the relay-timing `fmodf` in `control_step.cpp` runs only while waiting for the zero crossing; the scope and the data dispatch use integer `%` per sample. Change in the library fork: θ wraps with one conditional step (`fmodf` kept as a fallback if θ is still out of range, never in normal operation); the window position and the leaving sample's index wrap with a comparison. Host L0-GFL unchanged (52 pass).

| Stage | Interrupt / period (mean) | Task tick worst | Lost | PLL section (mean) | Sync window (mean) |
|---|---|---|---|---|---|
| OFF_IDLE | 56.7 → 56.3 (−0.4) | 52.1 → 51.2 | 0 → 0 | 2.12 → 1.71 | 1.54 → 1.52 |
| CONNECT_g | 71.3 → 70.8 (−0.5) | 66.6 → 66.1 | 0 → 0 | 2.11 → 1.70 | 1.54 → 1.52 |
| FOLLOW_CONN_G | 72.2 → 71.7 (−0.5) | 69.0 → 68.4 | 0 → 0 | 2.11 → 1.70 | 1.54 → 1.52 |
| other stages | −0.4 to −0.5 | −0.4 to −1.6 | 0 → 0 | −0.41 | −0.02 |

The float modulo is worth replacing (0.41 µs); the integer ones are not worth much (0.02 µs: the M4 divides in hardware).

### All four together (`5fcf14b`, campaign `2026-11-12_5fcf14b_153135_NtCZxW`, against `23afd86`)

| Stage | Interrupt / period (mean) | Task tick worst | Gap worst | Lost | Writes (mean) | Scope (mean) |
|---|---|---|---|---|---|---|
| OFF_IDLE | 56.7 → 50.4 (−6.3) | 52.1 → 45.1 | 117.6 → 121.1 | 0 → 0 | 2.76 → 1.44 | 2.91 → 1.32 |
| OFF_BUSREG | 63.2 → 53.9 (−9.3) | 58.8 → 50.7 | 111.6 → 111.5 | 0 → 0 | 7.44 → 3.71 | 3.82 → 1.61 |
| OFF_MPPT | 64.0 → 54.8 (−9.2) | 59.2 → 48.6 | 111.7 → 110.0 | 0 → 0 | 7.26 → 3.66 | 3.84 → 1.61 |
| SETTLE_c | 67.2 → 57.8 (−9.4) | 62.7 → 51.6 | 113.6 → 112.7 | 0 → 0 | 10.83 → 7.14 | 3.52 → 1.27 |
| CONNECT_g | 71.3 → 58.2 (−13.1) | 66.6 → 53.1 | 120.6 → 111.9 | 0 → 0 | 10.81 → 7.11 | 7.13 → 1.31 |
| INJECT_n | 72.1 → 59.8 (−12.4) | 64.2 → 53.5 | 115.6 → 116.3 | 0 → 0 | 10.93 → 7.23 | 6.26 → 1.19 |
| VAC_FOLLOW_a | 69.5 → 60.2 (−9.3) | 64.4 → 53.5 | 115.6 → 113.5 | 0 → 0 | 10.95 → 7.26 | 3.81 → 1.58 |
| FOLLOW_CONN_G | 72.2 → 59.0 (−13.2) | 69.0 → 56.1 | 114.5 → 112.4 | 0 → 0 | 10.81 → 7.11 | 7.22 → 1.34 |
| OPEN_LOOP_o | 70.3 → 61.1 (−9.2) | 65.4 → 55.3 | 117.5 → 121.3 | 0 → 0 | 10.97 → 7.27 | 3.37 → 1.12 |

**Verdict.** The four gains add up: alone they sum to about −10.5 µs on average, together −10.15 µs (changes 1 and 2 both shorten the scope part, so they overlap a little). The worst task tick is 45.1–56.1 µs (budget 80 µs: 24 µs of margin in the worst stage, G). With the task API's mean 10.8 µs in the same interrupt, the interrupt's worst is about 56–67 µs per period, so the 80 µs budget holds even if it is read as the whole interrupt. No period lost, background tasks served (telemetry every 102 ms). Against the code before any change (`a528915`, without the write probes): −9.82 µs.

| Change | Alone (mean over stages) | Where it shows |
|---|---|---|
| 1 Load bookkeeping | −6.97 µs | every stage |
| 2 Scope 2 kHz by default | −1.13 µs | g, n, G (−2.9 to −3.7 µs); mean only, not the worst tick |
| 3 Boost start / burst on change | −1.99 µs | stages with the boost running (−2.2 µs); 0 on the bench (boost idle) |
| 4 Modulos | −0.44 µs | every stage |
| **All** | **−10.15 µs** | |

**Next, by size:** the ADC path (task API data dispatch 9.4 µs + sensor reads 8.1 µs per period), with Jean; proposal 3b (bridge duty API, about 3 µs in every bridge stage), with Jean and Clément.
