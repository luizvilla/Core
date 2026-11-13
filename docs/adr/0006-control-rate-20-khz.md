---
status: accepted
date: 2026-11-13
---

# Control rate 20 kHz (critical task every 50 µs)

The critical task ran every 100 µs (10 kHz). Andres Vazquez Sieber set it to 50 µs in `be933ec` after simulating the controllers at 20 kHz. On branch `20khz_test` the team works at 20 kHz from now on (Luiz Villa, 2026-11-13).

## Decision

- `app::control_task_period = 50` µs. The hot loops (boost current, inverter current, PLL, observer) run at 20 kHz. The cold loop keeps `cold_loop_divider = 10`, so it runs at 2 kHz (was 1 kHz). Every controller gets its sampling time from `app::hot_loop_period` or `app::cold_loop_period`, so no gain was retuned by hand.
- **Durations are written in seconds and converted to ticks** with `app::ticks(seconds)`. They no longer change with the rate:

  | Constant | Duration | Ticks at 50 µs (100 µs before) |
  |---|---|---|
  | `grid_voltage_stale_ticks` (from `grid_voltage_stale_time`) | 20 ms | 400 (200) |
  | `relay_contact_ticks` (from `relay_contact_time`) | 20 ms | 400 (200) |
  | main.cpp `v_ac_rms_window_sample_count` | one 50 Hz period | 400 (200) |
  | main.cpp `trip_test_delay_ticks` | 100 ms | 2000 (1000) |
  | main.cpp scope dividers | 2 kHz, 10 kHz with key `v` | 10 and 2 (5 and 1) |

- **Inverter library** (`single_phase_inverter`, commit `2933cfe`):
  - The synchronization window, one grid period of samples, holds up to 512 samples (was 256). At 50 µs a period is 400 samples, 421 at 47.5 Hz. With 256 the average covered 12.8 ms, the harmonics no longer averaged out of |Vq|, and the inverter never synchronized on the bench grid (L0-APP-09).
  - The synchronized dwell before connecting is 100 ms, counted from Ts at `init()`. It was 1000 calls, which is 50 ms at 20 kHz.
- **Tests follow the rate instead of assuming 100 µs:**
  - L0-APP takes its ticks per second from `app::control_task_period`.
  - The L0-TIM harness and `evaluate.py` take the period from `control_step.h`. The budget stays 80 % of the period (ADR 0005), so it is now 40 µs, and the largest gap between task starts is 1.5 periods (75 µs).
  - The L1 contract has `control_period_s` 5e-5. L1-INT-02 and L1-APP-01/08 derive their tick counts from it.

## Considered options

- **Keep 10 kHz.** This is the rate that is validated on the bench (steps 1 and 2b) and in Jean Alinei's switched L1 runs of 2026-11-05. Not chosen: the team moves to the 20 kHz controllers.
- **Leave tick counts as numbers and change them by hand.** That is how the 100 µs assumptions were hidden in the first place. Not chosen.
- **Keep the 2 kHz scope rate in ticks (divider 5).** That would give 4 kHz and halve the capture window. Not chosen.

## Consequences

- **CPU time is the main risk.** At 100 µs the worst critical tick was 29.9–38.9 µs (L0-TIM after optimization 6 and the merge of Jean's boost work), against a 40 µs budget now. L0-TIM-01/02/03 must be run at 50 µs before any bench session at 20 kHz.
- The cold loop and the boost burst (the skip count moves by one per hot loop) now react twice as fast in time. MPPT perturbs at 2 kHz.
- Values still tuned or measured at 10 kHz are to be checked at 20 kHz:
  - `vac_follow_advance` (1.3 periods, bench-tuned)
  - the L1 bridge update delay (assumed 50 µs, was 80 µs)
  - the design studies in `tests/l1/design/` (`TS = 1e-4`)
  - Jean's archived switched results and the model tests L1-MOD-04/05, all run at 100 µs
- Jean's archived switched L1 results (`tests/out/switched-dcdc-*`) are evidence for 10 kHz only.
