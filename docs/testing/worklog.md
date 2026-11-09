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

