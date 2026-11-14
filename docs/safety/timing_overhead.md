# Safety API timing overhead

What the Safety API ([ADR 0009](../adr/0009-safety-api-hardware-protection.md)) costs the control task, measured before and after the changes. This is a living document: the plan, the decisions, then the results and findings as each step lands.

Issue: luizvilla/Core#27 · tracking: luizvilla/Core#14

## Why

The safety work so far measured how fast the protection **reacts**. It did not measure what the protection **costs** the control task. On a converter that cost is a safety question too: a control task that overruns degrades the control loop and can trip the watchdog.

What the Safety API adds:

| Where | What | Before this work |
|---|---|---|
| Control interrupt, every period, before the user task | old software check (only when sensors are watched) | measured on the micro-inverter (L0-TIM-03 `3:safety`) |
| Control interrupt, every period, after the user task | threshold read-back, program-flow checkpoints, frozen-sensor check, watchdog feed | not measured |
| System work queue, every 10 ms | run-time self-test slice (longest 429 µs) | duration only (L0-STK-02.b) |
| Inside that slice | RAM test block (16 words) with **all interrupts off**: delays the control task and the fault interrupt | not measured; estimated 8–9 µs from the start-up RAM test speed |
| Each output enable | gate asked again with interrupts locked (#22 race fix) | not measured |
| First output enable | arming of the hardware trips, self-tests included | not measured; milliseconds |

IEC 60730-1 Annex H sets no number for this, but the self-tests must not impair the control function, and the declared fault reaction times must hold under the worst-case load.

## Decisions

- **2026-11-14, two steps** (Luiz Villa):
  1. time the Safety API alone on a bare Spin (board-agnostic, goes upstream);
  2. integrate it into the micro-inverter and time it with that suite, before and after.
- **Limits: measure first.** Provisional limits, judged in step 1, to be confirmed with the numbers:
  - safety overhead per control period ≤ 5 % of the period;
  - longest interrupts-off window caused by the Safety API ≤ 2 µs;
  - both at 20 kHz (50 µs).
- **Step 2 branch:** `usolarverter-safety`, from `apf_quadrature`.

## Step 1: the Safety API alone (branch `safety-hw`)

### Method

**Builds** (one timing firmware, Twist 1.4.1 devicetree, bare Spin):

| Build | What | Purpose |
|---|---|---|
| B0 | Core `main` at `8111a0dc`, where `safety-hw` started: only the old Safety API | before |
| B1 | `safety-hw`, production | after |
| B2 | `safety-hw`, development mode, run-time self-tests off, `disarm()` | separates the per-period checks from the self-tests |

B0 is built in a temporary git worktree at `main` (which has no `OWNTECH_APP_DIR` hook), with the firmware copied to its `src/`.

**Scenarios**:
- S0: no safety call, as in the examples;
- S1: old software check watching every Twist sensor (`initShield()`).

**Rates**: 50 µs (20 kHz, the micro-inverter) and 100 µs.

**Load**: a calibrated busy loop in the control task, with data dispatch and PWM running.

**Measured in every build** (by the test firmware itself, no library probe):
- task-start jitter over many periods: worst case, spread, histogram;
- lost periods;
- total interrupt load, by subtraction: a background loop counts the cycles it gets.

**Measured in B1 and B2 only** (library probes, test builds only):
- cost of each block of the control interrupt;
- longest interrupts-off window;
- arming time at the first output enable;
- duration of a full RAM self-test pass under load.

**Reaction times under load**: software trip ≤ 5 periods + 2; hardware trip unchanged.

### Criteria (L0-TOV)

| ID | What |
|---|---|
| L0-TOV-00 | harness failure before a criterion |
| L0-TOV-01 | safety cost per control period, per rate |
| L0-TOV-02 | longest interrupts-off window caused by the Safety API |
| L0-TOV-03 | task-start jitter, B0 against B1 |
| L0-TOV-04 | no lost period |
| L0-TOV-05 | interrupt load, B0 against B1 |
| L0-TOV-06 | reaction times under load |
| L0-TOV-07 | arming time, RAM self-test pass time (info) |

### Commits

| # | Commit | Status |
|---|---|---|
| 1.1 | `feat(safety): timing probes and interrupts-off record` | planned |
| 1.2 | `test(safety): L0-TOV timing firmware and envs` | planned |
| 1.3 | `test(safety): L0-TOV harness, report, catalogue` | planned |
| 1.4 | `perf(safety): …` one commit per fix, if the numbers call for it | planned |
| 1.5 | `docs(safety): declared overhead` | planned |

## Step 2: the micro-inverter (branch `usolarverter-safety`, from `apf_quadrature`)

The before/after uses the micro-inverter's own suite L0-TIM ([ADR 0005](../adr/0005-on-target-timing-tests.md) on `apf_quadrature`):
- L0-TIM-01 runs `controlStep()` with interrupts locked, so it should show no change;
- L0-TIM-02 (real-time tick) and L0-TIM-03 (running firmware: `3:safety`, `3:dispatch`, gaps, lost periods) show the safety cost on the real 20 kHz tick, budget 40 µs.

| # | Commit | Status |
|---|---|---|
| 2.0 | before campaign: L0-TIM-01/02/03 on `apf_quadrature` | planned |
| 2.1 | `test(timing): probe the code after the user task` (on `apf_quadrature`, before the merge; before campaign again) | planned |
| 2.2 | `merge: safety-hw` (conflicts: `user_task_proxy`, `power-leg.yaml`, `GLOSSARY.md`, `catalogue.json`, CMake hooks) | planned |
| 2.3 | `feat(uSolarVerter): safety devicetree` (limits, exceptions, safe states, safe outputs; **Luiz Villa reviews the relay and safe states**) | planned |
| 2.4 | `feat(app): Safety API fault in telemetry and menu` (optional) | planned |
| 2.5 | after campaign, `compare.py` before against after | planned |
| 2.6 | `test(timing): baseline after the Safety API` (only if within budget) | planned |
| 2.7 | `perf(safety): …` (on `safety-hw` first, merged again) | planned |
| 2.8 | `docs(timing): before/after the Safety API` | planned |

## Results and findings

None yet.
