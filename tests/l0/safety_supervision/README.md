# L0-SUP · CPU supervision on the Spin

Checks that the converter reaches its safe state when the CPU itself fails ([ADR 0009](../../../docs/adr/0009-safety-api-hardware-protection.md), issue luizvilla/Core#16): the independent watchdog, the fatal-error handler and the HRTIM system fault, all armed when the first PWM output is enabled.

## Run

```bash
bash tests/common/target_preflight.sh            # bare Spin with MCUboot, ST-Link, OpenOCD
bash tests/l0/safety_supervision/run.sh --run    # build env SAFETY_SUP, flash, run every case (about 40 s)
tests/run_all.sh --l0 --target                   # or as part of a campaign
```

Hardware: a **bare Spin** (no shield, no power stage) and an ST-Link on its SWD header. No scope. The board then holds the SAFETY_SUP firmware: flash env USB again before a bench session.

## How it works

The test firmware ([`tests/firmware/safety_supervision`](../../firmware/safety_supervision/main.cpp), env `SAFETY_SUP` in [`tests/firmware/pio_tests.ini`](../../firmware/pio_tests.ini)) runs PWM units A, C, D, E, F at 200 kHz (B is left alone: it shares PA11/PA12 with USB). [`run.py`](run.py) drives it over SWD:

1. writes a command `(sequence << 8) | case` into `g_safety_sup_cmd`;
2. the firmware runs the case; a case that provokes a fault writes a *pending* mark in a TAMP backup register first;
3. after the reset, the firmware finds the pending mark, reads the latched fault record, checks that the PWM is refused while latched and comes back after `clearFault()`, stores the results in backup registers and acknowledges;
4. `run.py` waits for the acknowledgement in `g_safety_sup_report` (riding through the resets), then judges.

Reaction times use the DWT cycle counter: the firmware marks the cycle count just before the fault; CPU supervision stamps it when the outputs are disabled; both survive the reset in backup registers.

| Case | What it provokes | Criteria |
|---|---|---|
| CONFIG | Arms CPU supervision (PWM on) | L0-SUP-01: SYSCFG_CFGR2, PVD, fault states, watchdog timeout |
| NO_FALSE_RESET | 2 s on the kernel-timer feed, 2 s on a 50 µs control task | L0-SUP-02 |
| SLOW_TASK | 2 ms control task | L0-SUP-03: timeout ≥ 3 periods, no reset |
| FATAL_STACK | Recursion into the MPU stack guard of a thread | L0-SUP-04 |
| FATAL_HARDFAULT | Undefined instruction in a thread | L0-SUP-05 |
| CONTROL_STUCK | Control task that never returns | L0-SUP-06 |
| IRQ_MASKED | Interrupts masked forever | L0-SUP-07 |
| LOCKUP | Fault with FAULTMASK set | L0-SUP-08 (.c, pin levels during the lockup, needs a scope) |
| SOFT_RESET_LATCH | `k_panic()`, then a software reset | L0-SUP-09 |

## Findings while building it (2026-11-14)

- **Zephyr logs before the fatal handler.** With the default `CONFIG_EXCEPTION_DEBUG=y`, 250 to 300 µs passed between the fault and `k_sys_fatal_error_handler()`, with the PWM still switching; with the dump off, still 100 to 150 µs. CPU supervision now wraps the two C entry points of the fault path (`-Wl,--wrap=z_arm_fault,--wrap=z_fatal_error`): **0.82 µs** (stack overflow) and **0.95 µs** (undefined instruction).
- **`task.stopCritical()` then `task.createCritical()` panicked** (TIM6 source): Zephyr asserts in `irq_connect_dynamic()` that the line is disabled, and the timer driver left it enabled. Fixed in `owntech_timer_driver`.
- **Fault messages never reach the console**: the CPU halts and the watchdog resets within 1 ms, before the console flushes. The fault record in the backup registers (reason, PC, LR, thread, IPSR, stack) is the diagnostic channel; `run.py` prints it with the thread's symbol.
