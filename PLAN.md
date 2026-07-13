# Integrate RMS's nested zero-latency preemption into Core's Task API

## Context

`RMS/src/main.c` is a standalone Zephyr research prototype proving that two
Zero-Latency Interrupts (ZLI) can be chained on STM32G4: a hardware-timer
task (`RMS_0`, TIM6, ZLI priority 0) and a software-triggered task (`RMS_1`,
`NVIC_SetPendingIRQ(55)`, ZLI priority 1) that `RMS_0` can cleanly preempt,
with no RTOS scheduling jitter. The README documents the hard-won NVIC
priority/subpriority findings that make this deterministic.

`Core` (one level up from `RMS`, the OwnTech Power API) already has a
`TaskAPI` (`zephyr/modules/owntech_task_api/`) with exactly two task classes:
- **Critical task** (`createCritical`/`startCritical`/`stopCritical`):
  one hardware-timer-driven (TIM6 or HRTIM) ZLI task, uninterruptible.
- **Background tasks** (`createBackground`/...): plain Zephyr threads,
  cooperative, only run when the critical task isn't executing — no
  real-time guarantee, subject to RTOS jitter.

There's a gap between these two: nothing today lets a user run a *second*
hard-real-time task that is slower than the critical task but still
deterministic, while letting the critical task preempt it. That's exactly
what RMS proves is possible. The goal is to fold that proven mechanism into
`TaskAPI` as a reusable feature, rather than leaving it as a one-off
prototype. RMS itself does not need to change — it stays as the validated
reference; all changes land in `Core`.

Design decisions already made with the user:
- Reuse `IRQn 55` (`TIM7_DAC_IRQn`) as the software-triggered interrupt
  line, exactly as RMS does. This reserves TIM7's IRQ vector: it must be
  documented that TIM7 can't be used via the generic timer driver
  (`owntech_timer_driver`) while this feature is enabled.
- The mechanism must work regardless of whether the critical task's source
  is TIM6 or HRTIM. Good news: both sources already funnel through the same
  `user_task_proxy()` in `uninterruptible_synchronous_task.cpp`, so hooking
  in there covers both without separate code paths.
- New API name: `task.createPeriodic(fn, period_us)` /
  `task.startPeriodic(id)` / `task.stopPeriodic(id)` — chosen by the user
  because it describes the general idea (register a task with a period,
  the system derives its priority from that period) rather than hardcoding
  the notion of "the second one".
- Scope for this first pass: exactly **one** additional periodic level
  (mirrors what RMS actually validated). Only one spare IRQ line
  (`TIM7_DAC_IRQn`) is reserved. Supporting more nested levels later needs
  more identified free IRQ vectors per board/shield and is out of scope now
  — call this out explicitly rather than over-building.

## Repos involved

`RMS` and `Core` are two separate git repositories (siblings under
`~/code/`). RMS's own source is not modified — it stays the validated
reference. All work happens in the `Core` repo, on the already-created
`RMS` branch (currently checked out, with one unrelated pre-existing
uncommitted change to `zephyr/CMakeLists.txt` — a shield version bump from
`twist_v1_4_1` to `twist_v1_4_2` — that is not part of this plan and should
be left alone / committed separately by the user).

This plan document itself is written to `Core/PLAN.md` on the `RMS` branch.
The step sequence below is written so each step ends with its own commit,
in order, so the work can be picked up in a later session just by running
`git log` on the `RMS` branch and resuming at the first uncommitted step.

## Implementation steps (each ends with a commit)

### Step 0 — write and commit this plan
Write this plan to `Core/PLAN.md` on the `RMS` branch. Commit (on its own,
separate from the pre-existing shield-version change):
`docs: add integration plan for periodic task API`

### Step 1 — Kconfig scaffolding
Add the `OWNTECH_TASK_ENABLE_PERIODIC_TASKS` option to
`zephyr/modules/owntech_task_api/zephyr/Kconfig` and bump
`CONFIG_ZERO_LATENCY_LEVELS` to `2` in `zephyr/prj.conf` (see "Kconfig"
section below for exact text). No behavior changes yet since nothing
references the flag. Verify `pio run -e USB` still builds.
Commit: `task_api: add Kconfig scaffolding for periodic tasks`

### Step 2 — periodic task scheduling core
Add `zephyr/modules/owntech_task_api/zephyr/src/periodic_synchronous_task.h`
and `.cpp` (see "New module files" below), and register the new source file
in `CMakeLists.txt`. Not called from anywhere yet, so it's inert but
compiles standalone. Verify build.
Commit: `task_api: implement periodic task scheduling core`

### Step 3 — chain into the critical task proxy
Hook `scheduling_periodic_task_tick()` into `user_task_proxy()` in
`uninterruptible_synchronous_task.cpp` (see "Hook into the existing
critical-task proxy" below). Still no public API, so still inert/no
behavior change when the Kconfig flag is off. Verify build both with the
flag off (default) and on.
Commit: `task_api: chain periodic task tick into critical task proxy`

### Step 4 — public API
Add `createPeriodic`/`startPeriodic`/`stopPeriodic` to `TaskAPI.h`/`.cpp`,
gated behind the Kconfig flag, mirroring how `createBackground` is gated.
This is the first commit where the feature is actually usable.
Commit: `task_api: expose createPeriodic/startPeriodic/stopPeriodic API`

### Step 5 — docs
Extend `docs/task_introduction.md` with the new section and example (see
"Docs" below).
Commit: `docs: document nested preemptible periodic task`

### Step 6 — hardware validation (may span its own session)
Temporarily enable the Kconfig flag and wire `createCritical` +
`createPeriodic` into a scratch copy of `Core/src/main.cpp`, flash to the
SPIN board, capture timestamps, and confirm the preemption chronograph with
`RMS/ploting_sched.py` (see "Verification" below). Once confirmed, add a
small commented-out usage snippet to `Core/src/main.cpp`, following the
existing commented-out `createCritical` example style (lines 65-71) —
don't leave the scratch/enabled version committed.
Commit: `task_api: add periodic task usage example to default app`

### 1. New module files
`zephyr/modules/owntech_task_api/zephyr/src/periodic_synchronous_task.h/.cpp`
(sibling to `uninterruptible_synchronous_task.*`), holding:
- `scheduling_define_periodic_task(task_function_t task, uint32_t period_us)`
  — validates `period_us` is a positive integer multiple of the critical
  task's configured period (reject and return `-1` otherwise, same
  validation style as `scheduling_define_uninterruptible_synchronous_task`
  in `uninterruptible_synchronous_task.cpp:133`), stores the ratio, and
  wires `IRQ_DIRECT_CONNECT(55, 1, periodic_task_proxy, IRQ_ZERO_LATENCY)`
  (ZLI priority 1, one level below the critical task's priority 0) —
  directly modeled on `RMS/src/main.c:126-127`.
- `scheduling_start_periodic_task()` / `scheduling_stop_periodic_task()` —
  enable/disable IRQ 55, mirroring `scheduling_start/stop_uninterruptible_
  synchronous_task` in `uninterruptible_synchronous_task.cpp:192-274`.
- `scheduling_periodic_task_tick()` — called once per critical-task tick;
  increments a counter and calls `NVIC_SetPendingIRQ(55)` when the counter
  reaches the stored ratio, resetting to 0 — this is `RMS/src/main.c`'s
  `sched_1`/`PER_1` logic (`RMS_0()`, lines 34-41), generalized.
- The ISR trampoline (`periodic_task_proxy`) clears the pending flag
  (`NVIC_ClearPendingIRQ(55)`) and calls the user function — mirrors
  `RMS_1()` (`RMS/src/main.c:78-111`), minus the timing-instrumentation
  code (that was RMS's own measurement scaffolding, not part of the
  mechanism).

Reuse the existing `task_status_t` enum and `task_information_t` pattern
from `scheduling_common.h` rather than inventing new bookkeeping types.

### 2. Hook into the existing critical-task proxy
In `uninterruptible_synchronous_task.cpp`'s `user_task_proxy()` (currently
lines 107-123), add a call to `scheduling_periodic_task_tick()` after the
safety/data-dispatch handling. Since both the TIM6 and HRTIM paths already
call this same proxy, this one hook covers both sources — no per-source
branching needed.

### 3. Public API
`TaskAPI.h` / `TaskAPI.cpp`: add, gated behind a new Kconfig flag (see
below), mirroring how `createBackground` is gated behind
`CONFIG_OWNTECH_TASK_ENABLE_ASYNCHRONOUS_TASKS`:
```cpp
int8_t createPeriodic(task_function_t periodic_task, uint32_t task_period_us);
void startPeriodic(int8_t task_number);
void stopPeriodic(int8_t task_number);
```
`createPeriodic` must be called after `createCritical` (the critical task's
period is the base unit the ratio is computed from).

### 4. Kconfig
In `zephyr/modules/owntech_task_api/zephyr/Kconfig`, add (alongside the
existing `OWNTECH_TASK_ENABLE_ASYNCHRONOUS_TASKS` block):
```
config OWNTECH_TASK_ENABLE_PERIODIC_TASKS
    bool "Enable support for a nested preemptible periodic real-time task"
    default n
    help
      A periodic task is an additional hard real-time task that runs less
      often than the critical task but remains a Zero-Latency Interrupt,
      meaning it can be preempted by the critical task without RTOS
      scheduling jitter. Enabling this reserves TIM7's IRQ vector
      (TIM7_DAC_IRQn) as a software-triggered interrupt line: TIM7 can no
      longer be used through the generic timer driver while this is on.
```
In `zephyr/prj.conf`, bump `CONFIG_ZERO_LATENCY_LEVELS` to `2` (currently
unset/default `1`) — same value RMS's own `prj.conf` uses at minimum,
needed so the second ZLI priority level actually exists.

### 5. Build wiring
`zephyr/modules/owntech_task_api/zephyr/CMakeLists.txt`: add
`src/periodic_synchronous_task.cpp` to `zephyr_library_sources()`.

### 6. Docs
Extend `docs/task_introduction.md` with a new section documenting
`createPeriodic`/`startPeriodic`/`stopPeriodic`, reusing RMS's README
explanation of NVIC priority vs. subpriority and Zero Latency Interrupts
(the "hardly obtained wisdom" section) since that context is what makes the
feature safe to use correctly. Include a minimal example:
```cpp
task.createCritical(fast_task, 50, source_hrtim);
int8_t periodic_id = task.createPeriodic(slow_task, 500); // fires every 10th fast_task tick
task.startCritical();
task.startPeriodic(periodic_id);
```

## Verification
1. Build with the feature untouched (`OWNTECH_TASK_ENABLE_PERIODIC_TASKS`
   default `n`) for both `pio run -e USB` and `-e STLink` in `Core/` —
   confirms zero regression to existing critical/background task behavior.
2. Temporarily enable the Kconfig, wire `createCritical` + `createPeriodic`
   into a scratch build of `Core/src/main.cpp` (or a copy), flash to the
   SPIN board (STM32G474, same MCU family RMS was validated on), and
   capture start/end timestamps the same way `RMS/src/main.c` does.
3. Feed that capture into `RMS/ploting_sched.py` and confirm the
   preemption chronograph looks the same as the ones already documented in
   `RMS/README.md` (clean nesting, no missed preemptions) — this is the
   direct end-to-end proof that the generalized, integrated version
   behaves identically to the validated standalone prototype.
