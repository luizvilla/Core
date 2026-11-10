# Test work log

A dated log of every change made to **production code** (anything under `src/` or `owntech/`) while building the test procedure. Test-only changes are not logged here; they are described by their commits.

The procedure tests existing features. It must not change behaviour. Each entry therefore states the change, why the tests needed it, and the evidence that behaviour is preserved.

## Entry format

```markdown
## YYYY-MM-DD — <short title>

| Field | Value |
|---|---|
| Commit | `<sha7>` |
| Files | `path/a`, `path/b` |
| Reason | Why the test procedure needed this change |
| Behaviour change | None / description |
| Evidence | Tests, campaign record, or hardware check that shows behaviour is preserved |
| Hardware check | Not done / done on <date> with <board/shield version>: <result> |

<free-form notes: anything a reviewer must know, e.g. changed order of operations inside a tick>
```

## Entries

## 2026-11-05 — Extract the control step from the critical task

| Field | Value |
|---|---|
| Commit | the `refactor(app): extract controlStep …` commit that adds this entry (parent `566d9b3`) |
| Files | `src/app/control_step.h` (new), `src/app/control_step.cpp` (new), `src/main.cpp`, `zephyr/CMakeLists.txt` |
| Reason | L0-APP and L1 must run the application's real control logic and tuning without the shield API ([ADR 0001](../adr/0001-firmware-in-the-loop-via-pulsim.md)) |
| Behaviour change | None intended. Two timing differences inside one tick, listed below |
| Evidence | Firmware builds (`pio run -e USB`) with the same warning set as a full rebuild of the parent; flash +256 B (146 916 → 147 172), RAM +7 B. `control_step.cpp` compiles on the host with `-Wall -Wextra -Werror`. L0 campaign unchanged: 69 pass, 1 fail (L0-GFL-03.b, pre-existing), 7 info. L0-APP tests on the seam follow in the next commit |
| Hardware check | Not done. Recommended before L1: flash, run `i → p → b → m → c → i` from the serial menu and compare telemetry with the parent commit |

**What moved, unchanged.** From `loop_critical_task()`: boost measurement and reference updates, mode-change handling, the cold-loop divider, the hot loop, inverter bus update and sync/disconnect request, `calculateDuty`, grid observer update with the stale-VAC flag, grid-quality gating and the grid angle. From `setup_routine()`: controller `init` calls, startup ramp rate, startup current ceiling, bus reference and burst bounds, same order and values. From the globals: timing constants, PID tuning, grid constants and the observer configuration, now `app::` constants in `control_step.h`. The three controllers and the per-tick state (`cold_loop_tick`, `previous_mode`, `grid_voltage_age_ticks`) are now members of `ControlState`.

**What stays in `main.cpp`.** Sensor reads with last-value hold, the serial menu (which only sets `mode`, `inverter_sync_requested`, `voltage_reference`), telemetry, the scope, and every `shield.power.*` call.

**Order of hardware calls in a tick.** Unchanged: on a mode change `stop(LEG1_LOW)`, `stop(LEG2_LOW)`; then, if the mode is not IDLE, `setDutyCycle` ×2 and `start`/`stop` per leg; then `stop(LEG1_HIGH)`, `stop(LEG2_HIGH)`.

**Timing differences inside a tick:**

1. On a mode change, the two boost legs are now stopped **after** the control computation instead of before it, so for the duration of that computation (tens of µs) they keep the previous period's duty. They are stopped before any new duty is written, as before.
2. The VAC staleness counter is now updated at the start of `controlStep` instead of during the sensor read. Nothing reads it in between, so its value at the observer update is the same.

The leg enables are read before `scope.acquire()` instead of after; the scope does not touch the boost controller, so the values are the same. Operator commands are captured once per tick at the call to `controlStep`; the communication task cannot preempt the critical task, so this matches the previous reads.

**Signature.** The procedure sketched `controlStep(Measurements, OperatorCommands)`; the implementation also takes the `ControlState` explicitly, so tests and the L1 firmware block can hold independent instances instead of sharing hidden globals.

## 2026-11-05 — Temporarily build against the library fix for finding #1

| Field | Value |
|---|---|
| Commit | the `build(app): temporarily fetch …` commit that adds this entry |
| Files | `src/app.ini` |
| Reason | Evaluate the proposed fix for finding [#1](https://github.com/luizvilla/Core/issues/1) (PLL never synchronizes away from 50.00 Hz) in the firmware and on hardware before it is merged upstream |
| Behaviour change | **Yes, intended and temporary.** `lib_deps` fetches `luizvilla/single_phase_inverter` branch `fix/sogi-frequency-adaptive` (`4378319`, PR jalinei/single_phase_inverter#1) instead of `jalinei/single_phase_inverter` `main` (`0f22378`). Both SOGIs run at the PLL estimate clamped to ±5 % of ω_ref instead of the fixed ω_ref |
| Evidence | Firmware builds (`pio run -e USB`). L0 campaign on the fetched copy: 85 pass, 0 fail, 11 info; the record lists #1 as ready to close. Measured effects: Vq ripple at 51 Hz 3.27 V → 0.039 V; cold-start synchronization at 47.6–52.4 Hz now passes; cold-start lock at 50 Hz 0.19 s → 0.29 s; app grid-qualified-to-synchronized 1.12 s → 1.14 s |
| Hardware check | Not done |

**Revert condition.** When the PR is merged into jalinei `main` (or replaced by another remedy), restore `single_phase_inverter = https://github.com/jalinei/single_phase_inverter`, delete `owntech/lib/USB/single_phase_inverter` so PlatformIO re-fetches it, run `tests/run_all.sh --l0`, close #1 with that campaign ID and mark it `Closed` in the findings register. If the PR is rejected, revert the same way and reopen the triage on #1.

**Re-fetching.** PlatformIO does not always refresh a git dependency when only the URL or branch changes; deleting the library folder forces it.

## 2026-11-05 — Raise the startup slew rate to 200 V/s

| Field | Value |
|---|---|
| Commit | the `feat(app): raise the startup slew rate to 200 V/s` commit that adds this entry |
| Files | `src/app/control_step.h` (`app::startup_ramp_rate` 10.0F → 200.0F) |
| Reason | Requested by Jean Alinei, agreed by Luiz Villa. L1 uses the switched plant only ([ADR 0002](../adr/0002-switched-plant-only.md)); at 10 V/s the full 0 → 400 V startup is a 40 s run, hours of simulation at 50 ns. At 200 V/s it takes about 2 s |
| Behaviour change | **Yes.** STARTUP raises the bus reference 20× faster: 0.2 V per 1 ms cold-loop call instead of 0.01 V; 0 → 400 V in about 2 s instead of 40 s. The startup current ceiling (1 A on leg 1) and every other constant are unchanged. Charging 120 µF at 200 V/s needs 24 mA at the bus, about 0.25 A from 40 V before losses, within the ceiling |
| Evidence | Firmware builds (`pio run -e USB`). L0 campaign `2026-11-05_7b3857d-dirty_200722_RXHcr3`: 85 pass, 1 fail (known, #2), 11 info; L0-APP-01 reads the new value through `controlInit`. L1 campaign `2026-11-05_7b3857d-dirty_200921_gZABaa`: 24 pass, 1 fail (L1-APP-03.a, #3, unrelated to the ramp), 5 info, 9 blocked, the same counts as before the change. L1-INT-03.d and L1-APP-02.a measure the new rate (0.2 V after the first cold call; ramp error < 5 µV over 50 ms); L1-APP-02.b peak leg-1 current 0.605 A ≤ 1 A |
| Hardware check | Not done. Before relying on it: on the board, run STARTUP from a discharged bus and check that ILow1 stays under the 1 A ceiling and the bus reaches 400 V without overshoot |

The `BoostConverter` class default stays 10 V/s (L0-BST-05 tests the class); only the application's value changes. L1 checks now read the rate from `control_step.h` (`tests/l1/app_constants.py`) instead of repeating it, so a later change is picked up by the tests. `tests/l1/scenarios/dcdc_startup_full.json` now runs 2.5 s instead of 40 s; it is still run on demand, so L1-APP-02.d stays `BLOCKED` in the default campaign.

## 2026-11-06 — Build against jalinei/single_phase_inverter main again (#1 merged)

| Field | Value |
|---|---|
| Commit | the `build(app): fetch single_phase_inverter from jalinei main again …` commit that adds this entry |
| Files | `src/app.ini`, `tests/l1/dependencies.lock.json` |
| Reason | Revert condition of the 2026-11-05 entry met: Jean Alinei merged the fix (jalinei/single_phase_inverter#1) into `main` as `bcb7e71` |
| Behaviour change | None compared with the fix branch: `bcb7e71` has the same tree as `4378319` (squash merge, empty diff). `lib_deps` fetches `jalinei/single_phase_inverter` again; the test lock pins `bcb7e71` |
| Evidence | L0 campaign on `bcb7e71`: 85 pass, 1 fail (known #2), 11 info; every #1 criterion passes (record lists #1 as ready to close). Campaign ID in the findings register (§11) |
| Hardware check | Not needed for this revert (same source as the 2026-11-05 build) |

The local checkout `owntech/lib/USB/single_phase_inverter` was moved to `bcb7e71` with its origin set to jalinei, instead of being deleted and re-fetched: same commit, same result.

## 2026-11-06 — Boost control from Andres Vazquez Sieber's bench controller (ADR 0003)

| Field | Value |
|---|---|
| Commit | the `feat(boost): current-driven burst …` commit that adds this entry |
| Files | `src/boost_converter/boost.h`, `src/boost_converter/boost.cpp`, `src/app/control_step.h`, `src/app/control_step.cpp`, `src/main.cpp`, `src/app.ini`, `tests/l1/dependencies.lock.json` |
| Reason | Luiz decided to ship Andres's bench-tested boost control ([ADR 0003](../adr/0003-boost-adopts-current-driven-burst.md)); values confirmed with Andres on 2026-11-06 |
| Behaviour change | **Yes, intended.** Light load: current-driven pulse skipping on leg 1 (HRTIM burst, period 200 PWM periods, ±1 per tick) instead of the 395/405 V pause of both legs. PIDs never reset by burst; STARTUP → BUSREG resets nothing. STARTUP: no ramp, total current clamped to 0.25 A, leg 1 only. Current reference bounds 0–11 A, leg 1 up to 5.5 A. Current PI Kp 0.002712, Ti 341.3 µs, duty 0.05–0.60; voltage PI Kp = 2.6/6 · VBusRef / VLow (clamped 390–410 V, 24–48 V) every cold loop, Ti 4.57 ms. Full burst above 440 V; trip above 460 V or 8 A per leg, latched until IDLE. Leg 1 PWM runs in every active mode (burst gates its periods); leg 2 only with demand. `control_library` now from luizvilla `PID_update` (`ba1d12d`, adds `setKp`/`setTi`/`setKi`; same PID otherwise) |
| Evidence | L0 written first and failing on the previous object (40 compile errors: the new API is missing). With the change: L0-BST 26/26 pass (05–08 new; three checks of 02.c and 04.b updated to the intended "leg 1 runs at zero demand"); L0-APP-01.a checks the new tuning, 02.a uses a bus 0.2 V below the reference so the stiffer voltage PID does not saturate; full L0: 83 pass, 1 known fail (#2). Firmware builds (`pio run -e USB`) |
| Hardware check | Not done. Repeat Andres's three bench captures (`docs/testing/bench/2026-11-06_andres-boost/`) with this firmware: start-up without load, load above the STARTUP clamp, load removed; then a load step in BUSREG |

The 100 Hz PR of Andres's code is not adopted (it is always on there, which looks unintended). Leg 1's PID is no longer reset when its reference reaches zero: at zero demand it runs at minimum duty and burst skips its periods.

## 2026-11-06 — single_phase_inverter PWM start-up (1b4fd06), application acknowledges bridge PWM

| Field | Value |
|---|---|
| Commit | the `feat(app): single_phase_inverter PWM start-up …` commit that adds this entry |
| Files | `src/app/control_step.h`, `src/app/control_step.cpp`, `tests/l1/dependencies.lock.json` (`src/app.ini` unchanged: it follows jalinei `main`, Luiz's decision) |
| Reason | Jean Alinei's library update: a connection request now starts the bridge PWM at neutral (PWM_WAIT), links the grid-matching waveform near a zero crossing once the application acknowledges PWM, and settles 10 grid periods (PWM_SETTLE) before requesting relay closure. Integrated on the inverter side only; the boost keeps ADR 0003 |
| Behaviour change | Library: new states and acknowledgement API (`setPwmEnabled`, `setPwmStartDutyTolerance`, `setPwmSettleCycles`). Application: none observable; it acknowledges the bridge PWM it applied last period (always off: synchronization only), so the PWM start-up states are unreachable as before. **Note:** jalinei `main` was rewritten: `bcb7e71` (the #1 squash merge) is no longer on it; the same fix is `916178d`. Because `app.ini` follows `main`, any clean firmware build already fetched this library |
| Evidence | L0 written first: L0-GFL-04 updated and L0-GFL-16 added fail on `bcb7e71` (33 compile errors: the API is missing). On `1b4fd06`: full L0 89 pass, 1 known fail (#2). Firmware builds (`pio run -e USB`). L1 campaign: see the commit |
| Hardware check | Not needed for the application (no observable change). The PWM start-up itself is checked when the application connects to the grid |

## 2026-11-06 — HRTIM low-duty PWM reversal fix (cherry-picked from Jean Alinei)

| Field | Value |
|---|---|
| Commit | `fefe392` "Fix low-duty HRTIM PWM reversal" (Jean Alinei, cherry-picked by Luiz Villa) |
| Files | `zephyr/modules/owntech_hrtim_driver/zephyr/public_api/hrtim.h`, `zephyr/modules/owntech_hrtim_driver/zephyr/src/hrtim.c`, `zephyr/modules/owntech_shield_api/zephyr/src/Power.cpp`, `zephyr/modules/owntech_spin_api/zephyr/src/PwmHAL.cpp` |
| Reason | A nonzero compare below the prescaler's hardware minimum reversed the PWM output at low duty. The driver now maps such compares to zero (zero taking precedence over near-full duty at small periods), centralizes the full-duty output swap, and routes SPIN, SHIELD and frequency rescaling through the same setter |
| Behaviour change | Yes, at the duty extremes only: compares below the hardware minimum give zero duty instead of a reversed output. The application's duties stay far from them (boost 0.05–0.60, inverter about 0.09–0.91) |
| Evidence | Author's validation (commit message): STLink build, 512 register/state checks across CKPSC 0–7, 1,608 MHO984 center-aligned settings across all prescalers, both APIs, buck and boost, including 880 zero-duty checks; single-cycle transitions uncharacterized. Here: firmware builds (`pio run -e USB`). Host tests (L0, L1) do not compile the board drivers and are unaffected; the L1 PWM adapter does not model these compare limits |
| Hardware check | Part of the ADR 0003 hardware check (burst on leg 1 switches the boost at minimum duty 0.05 in short bursts) |

## 2026-11-06 — Minimum duty 0.005 and leg-1 dead time (Luiz Villa)

| Field | Value |
|---|---|
| Commit | the `fix(app): minimum duty 0.005 …` commit that adds this entry |
| Files | `src/app/control_step.h`, `src/main.cpp`, `tests/l1/contract.json` |
| Reason | Luiz's edits: the minimum PWM duty of every leg goes from 0.05 to 0.005, as in Andres Vazquez Sieber's controller (`duty_cycle_min_floor`); `setup_routine()` set leg 1's dead time from `leg2_low_deadtime_ns` (both 100 ns, so no difference at start-up) and now uses `leg1_low_deadtime_ns` |
| Behaviour change | Yes: the current PIDs' lower duty bound and the shield's minimum (`setDutyCycleMin(ALL, …)`) are 0.005, on every unit (boost at 200 kHz, inverter at 50 kHz). Burst starts on leg 1 when its PID sits at 0.005 instead of 0.05. The dead-time change has no effect until the menu changes one leg's dead time |
| Evidence | L0: 91 pass, 1 known fail (#2) and the 6 checks of L0-BST-09 (#5); no other test changed verdict. The L1 contract's `duty_min` set to 0.005 so the PWM adapter applies the same minimum. L1 not rerun in this commit |
| Hardware check | Not done. 0.005 at 200 kHz is 25 ns of on-time, below the 100 ns dead time: check on the board that leg 1's minimum-duty pulses in burst behave (with the HRTIM low-duty fix `fefe392`) |

## 2026-11-09 — Inverter bench stages and the grid relay (bench session)

| Field | Value |
|---|---|
| Commit | the `feat(app): inverter bench stages …` commit that adds this entry (on top of Jean Alinei's 2026-11-08 commits `9810a10`, `0be4f72`, `3743816`, `20c6073`, `6739371`) |
| Files | `src/app/control_step.h`, `src/app/control_step.cpp`, `src/main.cpp`, `tests/l1/firmware_block/wrapper.cpp` |
| Reason | Luiz's first inverter bench session ([sheet](bench/inverter-session.md)): generate 230 V RMS with no grid, hold PWM_SETTLE with the relay open, close the relay, inject a fixed current |
| Behaviour change | **Yes, intended.** `OperatorCommands::sync_requested` becomes `inverter_stage` (0 OFF, 1 SETTLE = Jean's PWM_SETTLE hold, 2 OPEN_LOOP, 3 CONNECT, 4 INJECT) with `inject_current`. OPEN_LOOP drives the bridge with a fixed 50 Hz modulation ramped (1 /s) to 325.3 V / VDCBus (max 0.95), relay open, the grid-following inverter unused. CONNECT sets 10 settling periods, so the inverter requests the relay after PWM_SETTLE; INJECT requests RUN with the d-axis reference capped at 0.1 A peak. The application commands the relay (`ControlOutputs::relay_close`) and takes the feedback from the hardware loop (`Measurements::relay_closed`). `main.cpp` drives GPIO 22 (set = closed, open at start-up) and reports the contact after 20 ms; menu keys `o`, `c`, `g`, `n`, `j`/`k`, `x`; `t` now selects the OFF stage instead of resetting GPIO 22 directly. An AC current above 1.5 A stops the bridge and opens the relay, latched until OFF. Telemetry adds stage, inverter state, relay, Id, Iq, P, Q, Id_ref; in CONNECT/INJECT the firmware scope triggers on the relay at 10 kHz, with IAC and the relay command replacing theta and grid quality |
| Evidence | L0: the new L0-APP-06…08 pass, Jean's L0-APP-04 passes with the stage encoding; full L0 99 pass, 7 known fails (#2, #5). Firmware builds (`pio run -e USB`). L1: the PWM adapter does not drive the bridge yet, so L1 runs that reach PWM_SETTLE stop (since Jean's `20c6073`); a bridge and relay adapter is the next L1 step |
| Hardware check | The bench session itself (L2-INV-10 pre-checks first: relay polarity) |

## 2026-11-09 — Manual relay test key (bench pre-check L2-INV-10.a)

| Field | Value |
|---|---|
| Commit | the `feat(app): manual relay test …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Luiz: check that the relay closes before running the grid steps |
| Behaviour change | Menu `R` toggles the relay on GPIO 22 alone. Accepted only with the inverter OFF, the boost IDLE, the bridge stopped and VAC below 20 V RMS (and below 30 V instantaneous); released by `R`, `x`, `i`, any stage key, or a grid voltage appearing. Hardware loop only (`main.cpp`); `controlStep` sees the relay feedback as usual and keeps the open-loop stage off while it is closed |
| Evidence | Firmware builds (`pio run -e USB`). Not testable at L0 (hardware loop) |
| Hardware check | It is the check: session pre-check L2-INV-10.a, on a dead bench |

## 2026-11-09 — Firmware scope: VDCBus and bridge duty, open-loop trigger

| Field | Value |
|---|---|
| Commit | the `feat(scope): VDCBus and bridge duty …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Luiz, bench step 0: the DC bus was not in the firmware capture, and the capture never triggered in the open-loop stage (it waits for Sync, which needs a grid) |
| Behaviour change | Capture: 12 channels (adds VDCBus_V and the applied bridge duty), 12 KiB. In the open-loop stage it triggers on the bridge start (2 kHz); other stages unchanged |
| Evidence | Firmware builds; ScopeMimicry buffer 256 × 12 × 4 B = 12 288 B, within its 65 535 B limit (static_assert) |
| Hardware check | Bench step 0 |

## 2026-11-09 — Firmware scope: forced trigger (menu f)

| Field | Value |
|---|---|
| Commit | the `feat(scope): forced trigger …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench step 1 never synchronized, so the capture (triggered by Sync rising) never fired and `r` returned the previous capture (finding #7) |
| Behaviour change | Menu `f` triggers the capture at once in any stage; otherwise unchanged |
| Evidence | Firmware builds |
| Hardware check | Bench: `q`, `c`, wait 2 s, `f`, `r` |

## 2026-11-09 — Bench calibration of VDCBus and IAC (this board)

| Field | Value |
|---|---|
| Commit | the `fix(sensors): bench calibration …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench: VDCBus read 96.4 V at 99.54 V (meter) and 357.4 V at 399.4 V; IAC read 0.30 A with no current. Both feed the open-loop index, the inverter duty scaling and the bench current trip |
| Behaviour change | Yes: after the sensors are enabled, the default linear conversion is composed with V = 1.14898·reading − 11.221 V for VDCBus, and −0.30 A on IAC. Board-specific (values measured on this board) |
| Evidence | Firmware builds. Two-point fit: 1.14898·96.4 − 11.221 = 99.54 V; 1.14898·357.4 − 11.221 = 399.4 V |
| Hardware check | Bench: VDCBus telemetry against the meter at 100 V and 400 V; IAC near 0 with no current |

## 2026-11-09 — Temporarily build against the library fix for finding #7 (period-averaged synchronization)

| Field | Value |
|---|---|
| Commit | the `build(app): fetch the single_phase_inverter fix for #7 …` commit that adds this entry |
| Files | `src/app.ini`, `tests/l1/dependencies.lock.json` |
| Reason | Bench step 1 never synchronized on the real mains (finding [#7](https://github.com/luizvilla/Core/issues/7)); Luiz asked to implement the averaging now and open the pull request later with the tests and the bench data |
| Behaviour change | **Yes, intended and temporary.** `lib_deps` fetches `luizvilla/single_phase_inverter` branch `fix/sync-period-average` (`9bfbaee`) instead of `jalinei/single_phase_inverter` `main` (`1b4fd06` + this commit). Synchronization is judged on Vd, Vq and ω averaged over one grid period (window of 2π/(ω̄·Ts) samples, at most 256, following the averaged frequency), with \|Vq\| below 1 % of the nominal peak instead of an instantaneous 0.1 V. SOGI gain unchanged (500): a gain of 50 makes the SOGI–PLL loop oscillate with the frequency-adaptive SOGI (host check, #7) |
| Evidence | Host, library alone: the bench grid never synchronized before (instantaneous \|Vq\| up to 58 V); with the fix, L0-GFL-10 (bench harmonics at 47.6–52.4 Hz, EN 50160 limits) and L0-GFL-13 (offsets, the bench grid as measured) pass, averaged \|Vq\| ≤ 0.055 V on the bench grid; all earlier L0-GFL pass, including the SYNC_LOST and PWM start-up tests. Full L0: 103 pass, 7 known fails. Firmware builds and installs the library at `9bfbaee` |
| Hardware check | Bench L2-INV-01 retest (menu `c`): telemetry state 1 → 9 → 8 |

**Revert condition.** When the fix is merged into jalinei `main` (pull request to open with these tests and the bench data), restore `single_phase_inverter = https://github.com/jalinei/single_phase_inverter`, pin the merged commit in the test lock, run L0, and close #7 with that campaign ID.

## 2026-11-09 — Telemetry: critical task load

| Field | Value |
|---|---|
| Commit | the `feat(telemetry): critical task load …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench: after `o`, the board became very slow and stopped answering; a capture taken after `c` shows the control step still at stage OFF (inverter state 0), so keys were not processed |
| Behaviour change | The critical task measures its run time (cycle counter) and the telemetry prints the longest run since the previous line (`load_us`, last field); no control change |
| Evidence | Firmware builds; RAM 30 % |
| Hardware check | Bench: `load_us` in each stage |

## 2026-11-09 — Critical task overrun: per-part timing, constant-cost averaging window

| Field | Value |
|---|---|
| Commit | the `fix(load): …` commit that adds this entry; library `fix/sync-period-average` at `3eac7db` |
| Files | `src/main.cpp`, `tests/l1/dependencies.lock.json` |
| Reason | Bench: `load_us` read 118 µs with the inverter OFF and 119 µs in `c`, above the 100 µs period; after `o` the telemetry stopped (background tasks starved) |
| Behaviour change | Library: the averaging window no longer recomputes its sums over all samples once per period (about 3000 cycles in one call); shadow sums replace them at constant cost. Firmware: a `load us:` line every 100 ms gives the longest run of the whole task and of each part (sensors, control step, PWM and relay application, scope) |
| Evidence | L0-GFL-10/13 results unchanged (averaged \|Vq\| ≤ 0.055 V); firmware builds |
| Hardware check | Bench: the `load us:` line in OFF, `c` and `o` |

## 2026-11-09 — Speed optimization; bridge outputs started once

| Field | Value |
|---|---|
| Commit | the `perf: speed optimization …` commit that adds this entry |
| Files | `zephyr/prj.conf`, `src/main.cpp` |
| Reason | Bench: the critical task runs 77–84 µs of its 100 µs period (control step about 53 µs) with the inverter OFF or synchronizing; the bridge application adds to it in the open loop |
| Behaviour change | Build optimized for speed (`CONFIG_SPEED_OPTIMIZATIONS=y`) instead of size (`-Os`). The bridge outputs are started once when the bridge starts and stopped once when it stops, instead of `start()` (or `stop()`) on both legs every tick; the duty is still applied every tick. No control change |
| Evidence | Firmware builds; flash 72 % → see the build log, RAM 30 % |
| Hardware check | Bench: the `load us:` line in OFF, `c` and `o` |

## 2026-11-09 — Telemetry: synchronization criterion and grid verdict

| Field | Value |
|---|---|
| Commit | the `feat(telemetry): synchronization criterion …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench: `c` held for a long time without synchronizing; captures taken without re-arming showed earlier events. The criterion itself is needed live |
| Behaviour change | A `sync:` line every 100 ms: averaged Vd, Vq, ω (with their limits), window full, grid observer verdict (`canOperate`) and trip reason. No control change |
| Evidence | Firmware builds |
| Hardware check | Bench: `sync:` line during `c` |

## 2026-11-09 — Grid observer judges one-period averages (finding #7, second part)

| Field | Value |
|---|---|
| Commit | the `fix(app): grid observer on one-period averages …` commit that adds this entry |
| Files | `src/grid_observer/gridObserver.h`, `src/grid_observer/gridObserver.cpp`, `src/app/control_step.cpp`, `tests/l0/app/test_app.cpp` |
| Reason | Bench, step 1 with the library fix: the inverter's averaged synchronization criterion held (Vd_avg 326–327 V, \|Vq_avg\| ≤ 0.45 V, ω_avg 313.9–314.4 rad/s), but `grid_ok` stayed 0 with no trip: the observer's RMS (from the instantaneous SOGI α/β) jumped between 206 and 256 V and kept restarting its 1 s qualification |
| Behaviour change | `GridObserver::updateEstimates(rms, hz, valid)` takes RMS and frequency directly (`update()` now calls it). Once the inverter's averaging window is full, the application feeds the observer RMS = averaged Vd / √2 and frequency = averaged ω / 2π. Detection after a grid step is delayed by up to one period (20 ms) on top of the 0.1 s trip delays |
| Evidence | New L0-APP-09: on the bench-measured grid the observer qualifies in 1.03 s and step 1 reaches PWM_SETTLE in 1.13 s; with the instantaneous feed it never qualifies (checked). L0-APP-05.d window widened from 0.10–0.12 s to 0.10–0.14 s for the averaging latency; L0-GOB (observer alone) unchanged. Full L0: no new failure. Firmware builds |
| Hardware check | Bench step 1: `sync:` line `grid_ok 1`, inverter state 1 → 9 → 8 |


## 2026-11-09 — Bridge leg order (output 180° from the grid) and VDCBus calibration redone

| Field | Value |
|---|---|
| Commit | the `fix(bench): bridge leg order …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench step 1: the inverter synchronized but its output was 180° from the grid (isolated probes on both ends). The schematic shows LEG2 drives L and LEG1 drives N, so with LEG1_HIGH as leg A, v_LN = −Vbus·(2D−1). Swapping only the init made the duty update fail (the driver accepts updates on leg A only: -EINVAL, PWM_UPDATE_FAILED), so it no longer synchronized. Separately, the meter read 399.4 V on the bus while the telemetry read about 435 V: the first calibration took its high point (357.4 V) from a capture at another bus voltage |
| Behaviour change | `initInverterUnipolar(LEG2_HIGH, LEG1_HIGH)` and `setInverterDutyCycle(LEG2_HIGH, …)`. VDCBus calibration from the meter: (99.54 V, 96.4 raw) and (399.4 V, 388.5 raw, back-computed from the 435.3 V reading with the old fit), so V = 1.02643 · raw + 0.592 V instead of 1.14898 · raw − 11.221 V |
| Evidence | Firmware builds (flash 80 %) |
| Hardware check | Bench step 1: output in phase with the grid, amplitude within ±3 %; telemetry VDCBus ≈ meter (≈ 399 V) |

## 2026-11-09 — Telemetry: inverter fault reason; scope triggers on a fault in CONNECT

| Field | Value |
|---|---|
| Commit | the `feat(telemetry): inverter fault reason …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench L2-INV-04 (dry run): from PWM_SETTLE (state 8), `g` put the inverter in FAULT (state 7) before the relay was commanded; the telemetry did not say which fault, and the CONNECT capture only triggers on the relay closing |
| Behaviour change | The `sync:` line ends with `fault <vsi_fault_reason_t>`. In CONNECT and INJECT the scope also triggers when the inverter enters FAULT, and `g` / `n` re-arm it (armed during `c`, it fills on the synchronization before `g`). No control change |
| Evidence | Firmware builds |
| Hardware check | Bench: `c`, `g`; read `fault`, download the capture |

## 2026-11-09 — Telemetry: critical task call rate; CONNECT capture rate selectable

| Field | Value |
|---|---|
| Commit | the `feat(telemetry): critical task call rate …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench L2-INV-04, capture 2026-11-09_20-24-38: about 6 ms after `g` (relay never commanded) the inverter faults. After `g`, VAC advances irregularly from one tick to the next (1–2 V steps then 25–30 V steps near the zero crossing, where a 50 Hz sine moves about 10 V per 100 µs tick), and the PLL frequency climbs from 310 to 337 rad/s while the grid stays at 50 Hz. Same PLL code as step 1, where it holds for minutes; `g` adds a 10 kHz capture and the library's settle counter (double precision, software-emulated on the M4) to a task already at about 80 µs of its 100 µs |
| Behaviour change | The `load us:` line ends with `rate <Hz>`, the measured call rate of the critical task (expected 10000). Menu `v` switches the CONNECT/INJECT capture between 10 kHz (default) and 2 kHz. No control change |
| Evidence | Firmware builds |
| Hardware check | Bench: `rate` in `c` and after `g`; `g` with `v` (2 kHz capture) |

## 2026-11-09 — Telemetry: longest gap between control periods, stale VAC periods

| Field | Value |
|---|---|
| Commit | the `feat(telemetry): longest gap …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench L2-INV-04: `g` from PWM_SETTLE gives fault 12 (SYNC_LOST) within 0.2–6 ms, on the board only (the same sequence on the bench-grid model passes, L0-APP-09.d). The call rate stays at 10 kHz over 100 ms, and VAC advances irregularly from one period to the next. Capture 2026-11-09_20-38-23 (10 kHz): at the first sample after `g` the PLL is already off (ω 342 rad/s, Vq 48 V) |
| Behaviour change | The `load us:` line ends with `gap max <µs>` (longest interval between two critical task starts, expected about 100) and `vac stale <n>` (periods without a new VAC sample, value held). No control change |
| Evidence | Firmware builds |
| Hardware check | Bench: both values in step 1 held, and on the line right after `g` |

## 2026-11-09 — Bench diagnostic: step 1 with the settle counter running (menu `e`)

| Field | Value |
|---|---|
| Commit | the `feat(bench): step 1 with the settle counter running …` commit that adds this entry |
| Files | `src/app/control_step.h`, `src/app/control_step.cpp`, `src/main.cpp` |
| Reason | Bench L2-INV-04: with the 2 kHz capture (capture 2026-11-09_20-41-43) `g` still gives fault 12 about 6.5 ms after the key, so the 10 kHz capture is not the cause. The only control difference between step 1 and step 2 before the relay is `setPwmSettleCycles(10)`: the library starts counting periods in PWM_SETTLE (a double-precision accumulation, software-emulated on the M4) |
| Behaviour change | `OperatorCommands::settle_count_test` (menu `e`, default off): in SETTLE, the settle count is `app::settle_count_test_cycles` (65535 periods, never reached) instead of 0, so the counter runs while the relay stays inhibited. Default behaviour unchanged |
| Evidence | Firmware builds; L0-APP: no new failure (L0-APP-03.c fails before and after this change) |
| Hardware check | Bench: `e`, then `c`: fault 12 in step 1 means the counting itself (or its timing) causes it; no fault means the cause is elsewhere in the switch to CONNECT |

## 2026-11-09 — Inverter library: single-precision settle counter (fault 12 after `g`)

| Field | Value |
|---|---|
| Commit | the `fix(deps): single-precision settle counter …` commit that adds this entry; library `luizvilla/single_phase_inverter` `b7fa96c` on `fix/sync-period-average` |
| Files | library `src/singlePhaseInverterFollowing.{h,cpp}`; `tests/l1/dependencies.lock.json` |
| Reason | Bench: from PWM_SETTLE, `g` gave fault 12 (SYNC_LOST) within about 6 ms, with the 10 kHz or the 2 kHz capture, `gap max` 105–120 µs and no stale VAC sample. With menu `e` (step 1 running the settle counter, relay still inhibited) step 1 faulted the same way; with `e` off it held. The counter was a `double` accumulation, software-emulated on the M4. How it disturbs the synchronization on the board is not explained; on the host the same code passes (L0-APP-09.d) |
| Behaviour change | `_pwm_settle_cycles` is a `float32_t`, incremented by ω·Ts/2π in single precision. It counts the same PLL periods; rounding over 10 periods is well under 0.01 period |
| Evidence | L0-GFL: no failure; L0-APP: no new failure (L0-APP-03.c fails before and after); firmware builds |
| Hardware check | Bench 2026-11-09, dry run L2-INV-04 (relay power contacts unwired): `g` goes PWM_SETTLE → GRID_CONNECT → GRID_CONNECTED, the relay closes about 9.7 ms after the command. Captures in `docs/testing/bench/2026-11-09_inverter/` |

## 2026-11-09 — Relay commanded ahead of a voltage zero crossing

| Field | Value |
|---|---|
| Commit | the `feat(app): relay commanded ahead of a voltage zero crossing` commit that adds this entry |
| Files | `src/app/control_step.h`, `src/app/control_step.cpp`, `tests/l0/app/test_app.cpp`, `tests/catalogue.json` |
| Reason | Bench L2-INV-04: the relay closes 9.7 ms after its command. Luiz Villa asked for the contact to close at a voltage zero crossing: the residual inverter/grid mismatch from an amplitude error is smallest there, and the contact switches at low voltage |
| Behaviour change | When the inverter requests the connection (GRID_CONNECT), the application issues the relay command once θ + ω̄·`app::relay_close_delay` reaches a zero crossing (θ = π/2 mod π, since v = Vd·cos θ when locked), ω̄ being the one-period averaged frequency; the command is then held while the inverter requests the connection. `app::relay_close_delay` = 9.7 ms; 0 restores the immediate command. The relay command comes up to half a period (10 ms) later than before; GRID_CONNECT's 1 s timeout is unaffected |
| Evidence | New L0-APP-07.d: contact instant within 1.8° of a zero crossing (limit 4°) at 4 start phases × 49.5/50/50.5 Hz; it failed before the change (11/36). L0-APP-07.a: settle window widened to 0.2–0.21 s. L0-APP-09.d passes; L0-APP: no new failure (L0-APP-03.c fails before and after). Firmware builds |
| Hardware check | Bench: CH4 on the relay command, a voltage probe on the grid side; the contact closes within about ±1 ms of a zero crossing (relay closing-time spread) |

## 2026-11-09 — Bench: protection test key (artificial 3 A for one period)

| Field | Value |
|---|---|
| Commit | the `feat(bench): protection test key …` commit that adds this entry |
| Files | `src/main.cpp`, `tests/catalogue.json` |
| Reason | Luiz Villa asked for a last dry-run check before connecting to the mains: the bench current trip exercised end to end on the board |
| Behaviour change | Menu `z` (amended the same day: it re-arms the scope with the bench trip as trigger, and the 3 A sample comes 100 ms later so the pre-trigger half is filled; the CONNECT trigger otherwise fires on the relay closing): one control period reads IAC as 3 A (above `app::bench_inverter_current_trip`, 1.5 A); the scope's IAC channel shows it. The trip logic itself is unchanged (covered by L0-APP-08). L2-INV-04 gets criterion .f |
| Evidence | Firmware builds |
| Hardware check | Bench L2-INV-04.f: `z` in GRID_CONNECTED → fault, bridge stopped and relay command dropped in the same period, latched until `x` |

## 2026-11-10 — Inverter current loop set from the application; inverter duty limit 0.95

| Field | Value |
|---|---|
| Commit | the `feat(app): inverter current-loop settings …` commit that adds this entry; library `luizvilla/single_phase_inverter` `ab35e2f` (`fix/sync-period-average`) |
| Files | `owntech/lib/USB/single_phase_inverter/src/singlePhaseInverterFollowing.{h,cpp}`, `src/app/control_step.{h,cpp}`, `src/main.cpp`, `tests/l1/dependencies.lock.json` |
| Reason | The current PI was hard-coded in the library (Kp 0.001 V/A, Ti 0.3 ms: no current control in practice, [design note](inverter-current-loop-design.md)). Luiz Villa asked for setters and getters so the values are set from the application and can be explored in experiments, and for the inverter duty limit 0.95 as a named constant (`duty_cycle_max_high_side`) |
| Behaviour change | Yes. Library: new setters/getters, defaults unchanged; the INVALID_CONTROL_STATE recovery now re-initializes the current SOGI with the configured gain instead of 500. Application: `controlInit()` sets candidate C (Kp 1 V/A, Ti 5 ms, limit 30 V, current SOGI 2000 rad/s, advance 0.5 period). `main.cpp`: LEG1_HIGH and LEG2_HIGH maximum duty 0.95 (Luiz Villa's two lines, now with the constant); with the 0.005 minimum the bridge duty spans 0.05–0.95 |
| Evidence | Firmware builds. L0: L0-GFL-17.a–e pass (new); the other L0 results are unchanged, L0-APP-03.c fails as before (finding #2). L1 (switched model, 2026-11-10): with today's library behaviour the connection trips at the contact whatever the gains (no current PI in GRID_CONNECT; through the bulb the PLL locks on the inverter's own voltage), see the design note §6; the C gains alone do not make step 2 pass |
| Hardware check | Not done |

## 2026-11-10 — Inverter voltage offset for the connection (bench step 1c)

| Field | Value |
|---|---|
| Commit | the `feat(app): inverter voltage offset …` commit that adds this entry; library `luizvilla/single_phase_inverter` `2f6616f` |
| Files | `owntech/lib/USB/single_phase_inverter/src/singlePhaseInverterFollowing.{h,cpp}`, `src/app/control_step.{h,cpp}`, `src/main.cpp`, `tests/l1/dependencies.lock.json`, `tests/l1/design/connection_l1.py` |
| Reason | Luiz Villa asked for a small offset on the inverter side so that current flows out of the inverter when the relay closes, adjustable from the bench with keys, never below the grid voltage (bench step 1c, L2-INV-05) |
| Behaviour change | Yes. Library: `setFeedforwardGain()` (0.5–1.5, default 1) scales the grid-voltage feedforward. Application: `OperatorCommands::vac_offset`, clamped to [0, 5 %], sets the gain 1 + offset every control step; default 0, so behaviour is unchanged until `+` is pressed. `main.cpp`: keys `+` / `-` (1 % of VAC per press), the offset in the `sync:` telemetry line |
| Evidence | Firmware builds. L0-GFL-18.a–c and L0-APP-10.a pass; other L0 results unchanged (L0-APP-03.c fails as before, finding #2). The L1-GFL-14 study patch follows the new library lines and still builds |
| Hardware check | Not done (L2-INV-05 is the check) |

## 2026-11-10 — Bench step 1d: the bridge copies VAC (test only, relay never closed)

| Field | Value |
|---|---|
| Commit | the `feat(app): bench step 1d …` commit that adds this entry |
| Files | `src/app/control_step.{h,cpp}`, `src/main.cpp` |
| Reason | Luiz Villa asked for a stage that synchronizes and drives the bridge from the measured VAC, so the inverter-side wave copies the grid with the smallest possible difference, without closing the relay (L2-INV-06) |
| Behaviour change | New stage `INV_VAC_FOLLOW` (key `a`): once the observer qualifies the grid, duty = 0.5 + k·(1 + offset)·(VAC extrapolated 1.3 periods − one-period VAC mean)/(2·VDCBus), k ramped 0 → 1 in 1 s; the inverter controller stays OFF (no relay request); bench trip active. The one-period VAC mean (200 floats, about 0.8 kB of RAM) runs in every stage. Other stages unchanged |
| Evidence | Firmware builds (RAM 30.1 %, flash 79.0 %). L0-APP-11.a–b pass; other L0 results unchanged (L0-APP-03.c fails as before, finding #2) |
| Hardware check | Not done (L2-INV-06 is the check) |

## 2026-11-10 — Step 1d: DC removal simplified (critical task overload reported)

| Field | Value |
|---|---|
| Commit | the `fix(app): step 1d DC removal …` commit that adds this entry |
| Files | `src/app/control_step.{h,cpp}`, `tests/l0/app/test_app.cpp`, `docs/testing/bench/inverter-session.md` |
| Reason | Luiz Villa lost control of the board with the critical task overloaded after flashing step 1d. The one-period VAC mean recomputed a 200-sample sum once per grid period (a few µs on that period, in every stage); it is the only addition of this change that could lengthen a period noticeably |
| Behaviour change | The VAC DC estimate is now a first-order filter (τ 2 s, two operations per period, no array, no loop); about 0.5 V of 50 Hz ripple left on the estimate; settled about 6 s after boot. RAM −0.8 kB |
| Evidence | Firmware builds (RAM 29.5 %). L0-APP-11.a–b pass. Cause of the overload not confirmed on the board yet: check the `load us` telemetry line |
| Hardware check | Pending |

## 2026-11-10 — Shorter serial telemetry

| Field | Value |
|---|---|
| Commit | the `feat(bench): short telemetry …` commit that adds this entry |
| Files | `src/main.cpp`, `docs/testing/bench/inverter-session.md` |
| Reason | Luiz Villa: the serial output is too verbose and the board answers slowly to `+` / `-` |
| Behaviour change | Telemetry only (background task): by default one short line every 500 ms (stage, inverter state, relay, VAC RMS, IAC, VDCBus, Id, Iq, grid_ok, fault, offset, critical-task load) instead of four lines every 100 ms; key `l` toggles the previous full telemetry; trip messages print once when they latch instead of every 100 ms. Control unchanged |
| Evidence | Firmware builds |
| Hardware check | Pending |

## 2026-11-10 — Step 2b: connection with the VAC copy, PI from the relay request, blanking and connect window

| Field | Value |
|---|---|
| Commit | the `feat(app): bench step 2b …` commit that adds this entry; library `luizvilla/single_phase_inverter` `8a145fc`, `80de00a` |
| Files | `owntech/lib/USB/single_phase_inverter/src/singlePhaseInverterFollowing.{h,cpp}`, `src/app/control_step.{h,cpp}`, `src/main.cpp`, `tests/l1/dependencies.lock.json`, `tests/l1/design/connection_l1.py` |
| Reason | Luiz Villa asked for a connection test that uses the VAC copy, starts the current PI before the contact and blanks the PLL around it, with the inverter counting from the relay command and moving to CONNECTED 60 ms later (L2-INV-07) |
| Behaviour change | Library: `setCurrentLoopFromConnect`, `freezePll`, `holdFeedforward`, `setExternalFeedforward`, `setConnectionBlanking` (from the relay command; alignment not judged while the PLL is frozen), `setConnectWindow`, `notifyRelayCommand`; all off by default, the existing L0 results unchanged. Application: stage `INV_FOLLOW_CONNECT` (key `G`) with the VAC copy as external feedforward, PI from the relay request, advance 0.3, blanking 15 ms, connect window 60 ms; the relay command notified to the inverter in every stage (no effect unless a blanking or window is set). The 60 ms blanking proposed was set to 15 ms after D-GFL-04 (longer blanks raised the peak from 0.37 to 0.76 A); the 60 ms is the connect window |
| Evidence | Firmware builds. L0-GFL-19.a–g and L0-APP-12.a pass; other L0 results unchanged (L0-APP-03.c and L0-BST-09 fail as before, findings #2 and #5). L1-GFL-14 rebuilt on the production library (no patch) |
| Hardware check | Not done (L2-INV-07 is the check) |

## 2026-11-10 — Bench trip 3 A; step 2b connected at 20 ms

| Field | Value |
|---|---|
| Commit | the `feat(app): bench trip 3 A …` commit that adds this entry |
| Files | `src/app/control_step.h`, `src/main.cpp`, `tests/l0/app/test_app.cpp`, `docs/testing/bench/inverter-session.md` |
| Reason | Luiz Villa: there is no relay feedback, so the grid is considered connected 20 ms after the command rather than 60 ms; and the bench current trip is raised to 3 A |
| Behaviour change | `app::bench_inverter_current_trip` 1.5 → 3.0 A (every grid and open-loop stage); `app::follow_connect_window` 60 → 20 ms (step 2b); the `z` protection test injects 4 A instead of 3 A so it still crosses the trip |
| Evidence | Firmware builds. L0-APP-08.a (2.9 A no trip, 3.1 A trip), L0-APP-11.b and L0-APP-12.a (CONNECTED 200 periods after the command) pass; other L0 results unchanged |
| Hardware check | Not done |

## 2026-11-10 — Critical task load back below the bench limit

| Field | Value |
|---|---|
| Commit | the `fix(load): …` commit that adds this entry; library `luizvilla/single_phase_inverter` (the commit it pins) |
| Files | `src/app/control_step.{h,cpp}`, `src/grid_observer/gridObserver.{h,cpp}`, `owntech/lib/USB/single_phase_inverter/src/singlePhaseInverterFollowing.cpp` |
| Reason | Bench (Luiz Villa): after today's changes `c` faulted SYNC_LOST and `a` tripped the grid observer (over-frequency) as soon as the bridge started. Full telemetry: critical task 76 µs at idle (control 55 µs) and up to 98 µs at the bridge start, call rate 9916 Hz (periods skipped): the PLL, which assumes 100 µs per period, loses step. Compared with yesterday's bench firmware (`acf1ad6`) the control path had grown by about 300 instructions per period (`calculateDuty` 311 → 466, `controlStep` 489 → 635): the per-period setters of steps 1c/2b and the connection features |
| Behaviour change | None intended. The step 2b settings and the feedforward gain are applied only when the stage or the offset changes; the VAC copy is computed only in steps 1d/2b; the library skips the connection bookkeeping when no relay command was notified; the grid observer counts its qualification and trip delays in integer samples instead of software `double` accumulation (same thresholds, computed once at init) |
| Evidence | Firmware builds; `GridObserver::updateEstimates` no longer calls software double routines (9 calls before). L0: grid observer, application and inverter suites pass as before (L0-APP-03.c, L0-BST-09 known) |
| Hardware check | Pending: the `load` field at idle and when the bridge starts |

## 2026-11-10 — Firmware scope records the synchronization averages

| Field | Value |
|---|---|
| Commit | the `feat(scope): …` commit that adds this entry |
| Files | `src/main.cpp` |
| Reason | Bench: step 2b (`G`) faults SYNC_LOST (12) within half a second while step 1 holds; the check judges the period-averaged Vd, Vq and ω, which the capture did not record |
| Behaviour change | Scope channels: `V_alpha_V` → `Vq_avg_V`, `V_beta_V` → `omega_avg_rad_s`, `VDCBus_V` → `Vd_avg_V` (the bus is in the telemetry). Same channel count. No control change |
| Evidence | Firmware builds |
| Hardware check | Pending: capture of the step 2b fault (`v` for 2 kHz, `G`, then `r`) |
