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
- **2026-11-15, limits after step 1.4** (Luiz Villa):
  - safety cost per control period ≤ **5 µs**, fixed at every rate (it replaces 5 % of the period: 2.5 µs at 50 µs was out of reach);
  - S1, the old software check on six sensors (about 10.5 µs per period), is **information only** for L0-TOV-01, -03 and -05. Speeding that check up is left for a later session;
  - the other limits stay provisional: interrupts off ≤ 2 µs, jitter ≤ +2 µs and CPU ≤ +5 points against B0.

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

**Load**: a busy loop of 10 µs in the control task, with data dispatch and PWM running (25 µs was planned; at 50 µs it overloads B1, see the findings).

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
| 1.1 | `feat(safety): timing probes and interrupts-off record` | done |
| 1.2 | `test(safety): L0-TOV timing firmware and envs` | done |
| 1.3 | `test(safety): L0-TOV harness, report, catalogue` | done (`d19b352`, test fixes `3e814b5`, `da48d77`) |
| 1.4 | `perf(safety): …` one commit per fix, if the numbers call for it | done: 1.4a–e; S1's old software check left for a later session |
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

### 2026-11-14, first measurements (probe build B1p, bare Spin, scenario S0)

Exploratory runs, before the harness. They are not campaign results; the campaign numbers will replace them.

Per period, safety-hw production with probes, 100 µs control task, 25 µs load:

| Block of the control interrupt | Mean | Max |
|---|---|---|
| old software check, no sensor watched | 2.11 µs | 2.13 µs |
| data dispatch (Core, not safety) | 18.1 µs | 20.5 µs |
| user task (test load) | 25.4 µs | 25.4 µs |
| threshold read-back | 3.81 µs | 3.82 µs |
| frozen-sensor check | 7.08 µs | 7.09 µs |
| watchdog feed and interrupt-rate check | 1.42 µs | 1.42 µs |

Other records:
- RAM self-test block with every interrupt off: **7.5–8.0 µs**;
- longest run-time self-test slice: 530–640 µs;
- first output enable (arming): 2.1 ms at 100 µs, 3.2 ms at 50 µs.

Findings:
- **The safety code costs about 14.4 µs per period**: 14 % of a 100 µs period, 29 % of a 50 µs one. That is far above the provisional 5 %, so step 1.4 will need optimisation. The heaviest blocks are the frozen-sensor check (7.1 µs for two current sensors) and the read-back (3.8 µs). Even the old software check costs 2.1 µs with nothing to check.
- **The flash runs at 8 wait states** (FLASH_ACR LATENCY = 8; 4 are enough at 170 MHz) on Core `main` and on `safety-hw`. The micro-inverter work on `apf_quadrature` found the same: 8 → 4 wait states shortened its tick by about 38 %, and moving the hot path to CCM SRAM removed more. B0 and B1 share it, so the before/after stays fair, but the absolute numbers here are inflated. Step 2 measures with `apf_quadrature`'s flash settings.
- **Overload ends in a watchdog reset.** At 50 µs with a 25 µs load, B1's control interrupt (≈ 50 µs) exceeds the period and the board resets on the independent watchdog. The watchdog is fed from the control interrupt on every period, so the mechanism is not yet explained; investigation pending. With a 10 µs load, B1 runs, and the control interrupt takes about 71 % of the CPU.
- **Bare-Spin artefact:** the floating current pins never change, so the frozen-sensor check fires after 2000 periods. In the timing builds `CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS` = 100000 keeps the check running at its usual cost without firing within the 4 s window.

### 2026-11-14, first full pass at 4 wait states (scratch run, before the campaign)

The flash is now at 4 wait states in every build (Luiz Villa's request): `safety-hw` carries `perf(spin)` (code of `59e55b9a` from `apf_quadrature`), and B0 gets the same patch (`tests/l0/safety_timing/flash_ws4.patch`).

| 50 µs task, S0 (no safety call) | B0 before | B1 after | B2 dev, self-tests off, disarmed |
|---|---|---|---|
| CPU taken from the main thread | 33.8 % | 73.6 % | 68.5 % |
| worst task-start deviation | 6.7 µs | 21.0 µs | 10.1 µs |
| longest stretch taken from the main thread | 24 µs | 692 µs | 83 µs |
| first output enable (arming) | 2 µs | 1.7 ms | 0.8 ms |

Per block (B1p, mean): old software check 1.47 µs (10.6 µs on six sensors in S1); read-back 2.56 µs; frozen-sensor check 4.99 µs; feed 0.92 µs. Total 10.0 µs per period (18.8 µs in S1). Data dispatch is 11.4 µs at 50 µs and 18.4 µs at 100 µs, but it is not safety code. RAM self-test block with interrupts off: 7.42 µs. Software trip under load: 230 µs from setChannelWatch() to stop (≤ 350 µs).

Findings:
- **The probed blocks do not explain the extra CPU.** B2 vs B0 is about 35 % at 50 µs, but B2's probed safety blocks are only about 15 %.
  - *Ruled out:* data dispatch volume. B0 and B2 have the same ADC triggers and sequences (registers read mid-window); only the watchdog-enable bit differs.
  - *Found by PC sampling in B1* (100 halts in the window): 55 % in the control interrupt, 34 % in threads, **9 % in SysTick**. The backup watchdog feeder is a kernel timer every 200 µs. It keeps firing while the control task is the one feeding. Its callback returns at once, but each tick goes through the kernel's timeout code, with spinlock validation (asserts are on in these builds). That code runs with interrupts masked, which plausibly also explains part of B1's jitter.
- **Test fixes:**
  - S1's old software check tripped on a floating sensor in B1 and B2. The scenario now records which limit, so it can be widened in the test overlay.
  - In B0, the old reaction (GPIO reconfiguration) left OENR set, so a trip there was invisible.
  - The REACT case now loads the devicetree thresholds before tightening V_HIGH. B2's 0x483 was a high limit of 0.

Optimisations, one commit each, re-measured after each:
- 1.4a: stop the feeder timer while the control task feeds;
- 1.4b: RAM self-test in 4-word blocks with interrupts re-enabled between them, same work per slice;
- then the frozen-sensor check, the old software check and the read-back.

### Campaign history (step 1)

| Campaign | Code | What it shows | Caveat |
|---|---|---|---|
| `2026-11-14_d19b352_232824` | harness (1.3), no optimisation | reference before the fixes | B0 converted nothing (no ADC trigger) and B0's S1 ran with stale V1/V2 limits: B0 is not like for like |
| `2026-11-14_c7b7fca-dirty_233430` | 1.4a + 1.4b (feeder timer, RAM blocks) | interrupts off 7.42 → 1.87 µs; B1 jitter at 50 µs 20.8 → 9.1 µs; B1 CPU 73.8 → 67.0 % | dirty: measured both fixes together; same B0 caveats |
| `2026-11-14_3e814b5_234539` | + ADC triggers in the test firmware | B0 now dispatches data like an application | B0's S1 still with stale V1/V2 limits (incremental worktree build) |
| `2026-11-14_da48d77_235337` | + clean builds | **first like-for-like campaign**, after 1.4a and 1.4b | — |
| `2026-11-15_a2d1ce9_000222` | + 1.4c, 1.4d, 1.4e | safety cost at 50 µs 10.04 → 4.52 µs mean; B1 CPU 67.3 → 56.2 % | judged against 5 % of the period |
| `2026-11-15_8276ed6-dirty_090549` | same code, limits of 2026-11-15 | **reference after step 1.4**: verdicts below | dirty: `zephyr/CMakeLists.txt` names shield v1.4.2, whose overlay differs from v1.4.1 only by its version string |

Like for like (`da48d77`), what the Safety API adds over Core `main`:

| | 50 µs | 100 µs |
|---|---|---|
| CPU, S0 | B0 49.3 % → B1 67.3 % (+18.0 pts, ≈ 9 µs per period) | 32.3 % → 41.6 % (+9.3 pts) |
| worst task-start deviation, S0 | 7.89 → 9.61 µs | 7.89 → 7.93 µs |
| CPU, S1 (old check on six sensors) | 67.1 % → 84.4 % | 41.2 % → 50.2 % |
| interrupts off (RAM block) | 1.88 µs | |
| software trip under load | 208 µs (≤ 350 µs) | |

Further findings:
- **The old API's reaction repeats every period, forever.** With a stale devicetree, B0's old software check tripped on the floating V1/V2 pins. PC sampling then showed `_open_circuit()`, `gpio_pin_configure`, `resetPin` and `PowerAPI::stop` running in the control interrupt on every period, about 13 µs each time and 98 % CPU at 50 µs. The old reaction never latches, and it takes the GPIO API's mutex in an interrupt every period. #22 replaced it with one latch and one emergency stop.
- **PlatformIO's stale-devicetree trap hit the test itself:** the B0 worktree built incrementally. The harness now clean-builds every env.
- **Without ADC triggers, Core `main` converts nothing,** while the Safety API's watchdog path adds its own trigger. The test firmware now sets ADC1 on PWMA and ADC2 on PWMC, as `docs/spin_dataAPI.md` shows.

Fixes 1.4c–e, committed separately (each touches its own probed block):
- 1.4c (`6fa34e1`): the frozen-sensor check compares raw samples and checks one sensor per period.
- 1.4d (`6086d17`): the software check returns at once when no sensor is watched.
- 1.4e (`a2d1ce9`): the read-back reads one trip path per period. **Declared detection time of a threshold changed behind the API: from 1 control period to N periods** (N = armed trip paths, 6 on Twist). Needs Luiz Villa's review.

### 2026-11-15, after step 1.4 (campaign `8276ed6-dirty_090549`)

| 50 µs task | B0 before | B1 after | Limit | Verdict |
|---|---|---|---|---|
| safety cost per period, S0 (worst; mean) | — | 4.65 µs; 4.51 µs | ≤ 5 µs | PASS |
| interrupts off (RAM block) | — | 1.88 µs | ≤ 2 µs | PASS |
| worst task-start deviation, S0 | 6.88 µs | 10.81 µs (+3.9) | ≤ +2 µs | FAIL |
| CPU taken from the main thread, S0 | 49.3 % | 56.2 % (+6.9 pts) | ≤ +5 pts | FAIL |
| software trip under load | — | 222 µs | ≤ 350 µs | PASS |

- At 100 µs every S0 criterion passes (4.65 µs; jitter 7.89 → 7.54 µs; CPU 32.3 → 36.0 %).
- Per block, mean: plausibility 1.98 µs, read-back 1.31 µs, feed 0.91 µs, old software check (no sensor watched) 0.44 µs. From step 1.3 to here: 10.0 → 4.5 µs.
- S1 (information): the old software check on six sensors costs 10.5 µs per period (14.5 µs safety in total), B1 CPU +8.6 pts and jitter +5.2 µs at 50 µs. It is the next speed-up, in a later session.
- Still open at 50 µs: task-start jitter and CPU load in S0 exceed their provisional limits.
