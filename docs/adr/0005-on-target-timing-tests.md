---
status: accepted
date: 2026-11-12
---

# On-target timing tests (L0-TIM) on the Spin through an ST-Link

The critical task runs every 100 µs. On the bench it already takes 74–75 µs with the bridge off and 82–88 µs with it on, and step 2b faulted with SYNC_LOST while a timing problem was suspected. Nothing tells a developer that a change made the task slower until the board misbehaves. L0 as defined in the procedure (§2) runs on the host with `g++`, and host timings say nothing about the Cortex-M4.

## Decision

- A group of timing tests, **L0-TIM**, keeps L0's character (open loop, synthetic inputs, one object or `controlStep` at a time) but **runs on the Spin**: the firmware is built in a PlatformIO env `TIMING` (flag `TIMING_HARNESS`) and flashed through an ST-Link with PlatformIO's OpenOCD.
- Time is counted with the Cortex-M4 DWT cycle counter (170 MHz, one register read, no interrupt lock).
- Results are written to a RAM structure (`g_timing_report`, `src/timing/timing_report.h`) and read over SWD while the core runs. The USB console takes no part in measuring.
- `TIMING_PROBE(i)` marks the sections of `controlStep`. It expands to nothing outside the TIMING env, so the production firmware, the host L0 tests and the L1 C-Block are unchanged.
- **Budget: 80 µs**, 80 % of the control period, for the whole critical task (L0-TIM-02, L0-TIM-03). It is set by the user on 2026-11-12. No document stated one; it is expected to fail at first and to show by how much.
- Per-section regression (L0-TIM-01): each section's worst case may not exceed the committed baseline (`tests/l0/timing/baseline.json`) by more than 10 % (plus 50 cycles). A change that legitimately costs more updates the baseline in the same commit and says why.
- A test that reflashes the board runs only when asked (`tests/run_all.sh --l0 --target`, or `tests/l0/timing/run.sh --run`); otherwise it reports BLOCKED. A preflight script checks the tools, the probe and the board and prints the fix for each missing item.

## Considered options

- **Host timing (g++ on x86, or QEMU):** no relation to the Cortex-M4's cycles, flash wait states and caches. Rejected.
- **The existing `load` telemetry over USB:** measures the running task, interrupts included, but cannot say which object is slow, depends on the console, and needs a person at the bench. It stays as a bench indicator; L0-TIM-03 will measure the same quantity unattended.
- **GPIO toggling and an oscilloscope or logic analyser:** precise, but manual and not repeatable as a test. Possible later for L0-TIM-04.
- **Cycle counter, results in RAM, read over SWD:** chosen.

## Consequences

- Procedure §2: the L0 row names the on-target tooling for L0-TIM. The catalogue gets a Timing tab.
- Running L0-TIM needs an ST-Link, a powered Spin with the MCUboot bootloader, and PlatformIO's OpenOCD; `tests/l0/timing/README.md` lists them, `preflight.sh` checks them.
- After a run the board holds the TIMING firmware (no tasks, no PWM); the USB env must be flashed again before a bench session.
- L0-TIM-01 rows are measured with interrupts locked and the code running back to back; in the real task, ISRs and colder caches add to them. That difference is L0-TIM-03's to measure, not L0-TIM-01's.
