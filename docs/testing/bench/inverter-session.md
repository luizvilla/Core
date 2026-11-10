# Inverter bench session

The first hardware tests of the inverter: generate 230 V RMS, synchronize with the grid, close the relay, inject a small current. Each step has its own firmware stage, selected from the serial menu, and its own record. The catalogue IDs are `L2-INV-00…06` (Inverter tab, level L2: on the board); the pre-checks are `L2-INV-10`.

| Step | Menu | Firmware stage | Inverter controller state | What it shows |
|---|---|---|---|---|
| 0 | `o` | `INV_OPEN_LOOP` | not used (OFF) | The bridge and filter make a 230 V RMS, 50 Hz wave with no grid |
| 1 | `c` | `INV_SETTLE` | PWM_SETTLE, held | The inverter-side voltage matches the grid before the relay |
| 1b | `g` | `INV_CONNECT`, relay power contacts unwired | PWM_SETTLE (10 periods) → GRID_CONNECT → GRID_CONNECTED | Dry run of step 2: the sequence and the relay timing, with nothing connected (L2-INV-04) |
| 1c | `c`, then `+`/`-` | `INV_SETTLE`, grid on the other side of the relay | PWM_SETTLE, held | The inverter-side voltage raised a little above the grid, so current flows out of the inverter at the contact (L2-INV-05) |
| 1d | `a` | `INV_VAC_FOLLOW`, grid on the other side of the relay | OFF (PLL running); the bridge copies VAC | The inverter-side wave copies the measured grid voltage sample by sample: the smallest difference the hardware allows, relay never closed (L2-INV-06) |
| 2 | `g` | `INV_CONNECT`, grid on the other side, offset from step 1c | PWM_SETTLE (10 periods) → GRID_CONNECT → GRID_CONNECTED | The relay closing disturbance, then zero current on the grid |
| 3 | `j`/`k`, `n` | `INV_INJECT` | RUN | A fixed d-axis current injected into the grid |
| stop | `x` (or `t`) | `INV_OFF` | STOPPING → DISCONNECT → OFF | The relay opens after the current is regulated to zero |
| relay test | `R` | `INV_OFF` (dead bench only) | OFF | The relay closes and opens on its own (pre-check L2-INV-10.a) |

Firmware: branch `test/microinverter-validation-v2`, the commit that adds this file or later. Build and flash with `pio run -e USB -t upload`. The stage logic is tested at L0 (`L0-APP-06…08`). The boost stays in IDLE for the whole session: the DC bus comes from the bench source.

## Bench setup

```
 DC source ──► DC bus ──► H-bridge ──► LC filter ──► [V inverter side] ──► relay (GPIO 22) ──► [V grid side, VAC sensor] ──► series bulb ──► mains
                                                          scope CH1                                scope CH2
```

- **DC source** on the bus: set the voltage (400 V) and a current limit (start at 0.5 A).
- **Mains through a series light bulb** (100 W incandescent suggested): it limits a fault current to about 0.45 A once hot, about 6 A cold at switch-on. It also makes the grid weak: the bulb drops 40 to 500 Ω times the injected current, which limits step 3.
- **Firmware sensors:** VAC is on the grid side of the relay; IAC is the line current. With the relay open, the firmware does not see the inverter-side voltage: CH1 of the scope is the only measurement of it.
- **Scope:** CH1 differential probe on the inverter side of the relay, CH2 differential probe on the grid side, CH3 current probe on the AC line (between the relay and the bulb), CH4 the relay command (GPIO 22) or the DC source current.
- **Serial terminal** at the board's USB port: `h` prints the menu. Log the session to a file (for example `pio device monitor | tee bench-inverter-$(date +%F).log`).

## Safety

- The bus and the AC side reach lethal voltages. Use differential probes only; never ground-reference the scope to the bus.
- **Stop:** `x` (inverter OFF: the current is regulated to zero, then the relay opens). **Emergency:** switch the DC source off and open the mains breaker; the bulb limits the fault current meanwhile.
- **Firmware protection:** an AC current above 1.5 A (`app::bench_inverter_current_trip`) stops the bridge and opens the relay, latched until `x`. The grid observer stops the grid stages if the grid leaves 207–253 V or 47.5–52.5 Hz.
- After the session: DC off, wait for the bus to discharge (check with a meter), unplug mains.

## Telemetry

**Default (short):** every 500 ms one line, `st <stage> inv <inverter_state> rly <cmd><fb> VAC <V RMS> IAC <A> Vbus <V> Id <A> Iq <A> ok <grid_ok> flt <fault> off <offset %> load <µs>`. Trip messages print once when they latch. Key `l` toggles the full telemetry below (every 100 ms), for when the extra fields are needed; keep it short while sending `+` / `-` so the menu stays responsive.

**Full (`l`):** every 100 ms one line, fields separated by `:`:

`ILow1 : VLow : duty1 : VAC : VAC_rms : grid_angle : IAC : VDCBus : boost_mode : burst_skip : stage : inverter_state : relay(cmd)(fb) : Id : Iq : P : Q : Id_ref : load_us`

- `stage`: 0 OFF, 1 SETTLE, 2 OPEN_LOOP, 3 CONNECT, 4 INJECT, 5 VAC_FOLLOW.
- `inverter_state`: 0 OFF, 1 VOLTAGE_SYNC, 2 GRID_CONNECT, 3 GRID_CONNECTED, 4 RUN, 5 STOPPING, 6 DISCONNECT, 7 FAULT, 8 PWM_SETTLE, 9 PWM_WAIT.
- `relay`: two digits, command then feedback (feedback = command held 20 ms).
- A `sync:` line: the synchronization criterion as the inverter judges it (one-period averages, finding #7): `Vd_avg` 293–358 V, `|Vq_avg|` < 3.25 V, `w_avg` 298.5–329.9 rad/s, `window` 1 once full, `grid_ok` (grid observer verdict) and `trip` (observer trip reason: 0 none, 1 invalid measurement, 2 UV, 3 OV, 4 UF, 5 OF), `fault` (inverter fault reason), `offset` (inverter voltage offset of step 1c, % of VAC). **A tripped observer stays tripped until the board restarts** (finding #2): power-cycle after any trip before retrying a grid step.
- A `load us:` line: the critical task's longest run (total and per part).
- `load_us`: the critical task's longest run since the previous line, in µs. Its period is 100 µs: above about 80 µs the serial menu and the telemetry slow down or stop.
- `Id`, `Iq` in A peak (dq frame of the grid voltage: Id in phase), `P` in W and `Q` in var from the inverter's measurements.

The DC bus voltage is the 8th telemetry field (`VDCBus`) and the DC source display; it needs no oscilloscope channel.

The firmware scope (`r` downloads, `q` re-arms) records 12 channels: VAC, V_alpha, V_beta, V_d, V_q, IAC, omega, Sync, relay command, inverter state, VDCBus, applied bridge duty. It triggers on Sync rising at 2 kHz (about 64 ms before and after); in the open-loop stage, on the bridge start at 2 kHz (re-arm with `q` before pressing `o`); in the CONNECT and INJECT stages, on the relay command rising at 10 kHz (12.8 ms before and after).

## L2-INV-10 · Pre-checks (before any power)

| # | Check | Expected | Result |
|---|---|---|---|
| a | **Relay, dead bench** (board powered, DC source off, mains unplugged; ohmmeter across the relay contacts, scope CH4 on GPIO 22):<br>1. At start-up: contacts **open** (GPIO 22 reset = open). If closed, the polarity is inverted: stop, swap `setPin`/`resetPin` for `relay_pin` in `main.cpp`.<br>2. Press `R` (manual relay test): the relay clicks, contacts **closed**, GPIO 22 high, telemetry `relay` 11 after 20 ms.<br>3. Press `R` again: contacts **open**, `relay` 00. | open → closed → open; the firmware refuses `R` unless the inverter is OFF, the boost IDLE, the bridge stopped and VAC below 20 V RMS, and opens it on any stage key or if a grid voltage appears. **Never with mains connected:** with the bridge off, a closed relay rectifies the mains onto the bus through the bridge diodes | Pass. the Relay is in open state by default|
| b | Menu `h` prints the inverter bench keys; telemetry runs; `stage` 0, `inverter_state` 0, relay `00` | as written | h works, telemetry runds stage is 0, inverter state is zero, relay is 00 |
| c | DC source: 400 V, current limit 0.5 A, output off | set | limit set at 410 protection at 450V |
| d | Bulb in series with the mains feed, mains breaker reachable | yes | yes there is a switch near the setup |

## L2-INV-00 · Step 0: open-loop 230 V RMS, no grid

**Conditions.** Mains unplugged from the AC terminals (the relay stays open in this stage, but the grid side must be dead for this step). DC bus from 100 V up to 400 V. Boost IDLE.

The index aims at 325.3 V / VDCBus and saturates at 0.95 below 342 V, so the output is 0.95·VDCBus/√2 up to there, then 230 V RMS:

| VDCBus | 100 V | 200 V | 300 V | 340 V | 400 V |
|---|---|---|---|---|---|
| Index | 0.95 | 0.95 | 0.95 | 0.95 | 0.81 |
| Expected RMS | 67.2 V | 134.4 V | 201.5 V | 228.4 V | 230 V |

**Sequence.**
1. Set the DC source to **100 V** first; switch it on; check `VDCBus` ≈ 100 V in the telemetry.
2. Press `o`. The index ramps from 0 to 0.95 in about 1 s, 50 Hz, no feedback.
3. Record CH1 (inverter side) over 100 ms: RMS, frequency, waveform shape; the DC source current. Everything except the voltage level is checked here (.b, .c, .d, .f).
4. Press `x`, raise the bus to 200 V, press `o`, record; then 300 V, 340 V, and **400 V** for .a. (Raising the bus while running is also possible: the index follows the measured bus at the next period.)
5. Press `x`. The bridge stops at once.

**Acceptance.**

| Criterion | Limit | Simulation reference | Result |
|---|---|---|---|
| .a Output RMS at 400 V | 218–242 V (±5 % of 230 V) | 226.7 V at m = 0.8 unloaded (L1-MOD-04: about 230 V at m = 0.81) | |
| .b Frequency | 49.9–50.1 Hz | 50.0 Hz | yes 50 Hz|
| .c No trip, relay command 0 throughout | yes | — | no trip command at 0 throughout|
| .d INFO: THD, DC source current (no-load losses), waveform shape near zero crossings | record | L1-MOD-04: THD < 2 % at m ≥ 0.6 | I do not have the current |
| .e INFO: RMS at 100, 200, 300 and 340 V against 0.95·VDCBus/√2 | record | 67.2 / 134.4 / 201.5 / 228.4 V | got similar values |
| .f At 100 V (first run): 50 Hz, clean sine shape, no trip, relay open, DC source current recorded | yes | — | The sine wave seems to saturate somehow |

Note: the L1 model switches the bridge at 200 kHz, the board at 50 kHz (finding #6): expect more ripple on the board.

## L2-INV-01 · Step 1: synchronized PWM, relay open (PWM_SETTLE)

**Conditions.** Mains connected through the bulb. DC bus at 400 V. Boost IDLE.

**Sequence.**
1. Check `VAC_rms` ≈ 230 V and `inverter_state` 0. The bulb stays dark (relay open).
2. Press `c`. After about 1 s (grid qualification) and 0.1 s (synchronization), `inverter_state` goes 1 → 9 (PWM_WAIT, bridge at neutral) → 8 (PWM_SETTLE, waveform linked near a zero crossing). It stays in 8.
3. Record CH1 (inverter side) and CH2 (grid side) on the same screen, over at least 3 periods: RMS of each, phase difference (cursor at zero crossings, or the scope's phase measurement), difference waveform (CH1 − CH2) if available.
4. Download the firmware capture (`r`): V_q near 0, Sync at 1, state 9 → 8.
5. Press `x`, or go on to step 2.

**Acceptance.**

| Criterion | Limit | Why | Result |
|---|---|---|---|
| .a RMS difference, inverter side − grid side | within ±3 % (about ±7 V) | Sets the relay closing current: about ΔV / R_bulb (cold) = 7 V / 40 Ω ≈ 0.2 A | the RMS is very close |
| .b Phase difference | within ±3° (about 0.17 ms) | 3° of 325 V is 17 V of instantaneous difference | now we have a problem with a 180 degree shift. **Fixed (`c935efb`):** LEG2 drives L on the board, so leg 2 is now leg A; capture 2026-11-10 00:10 shows both sides in phase (`docs/testing/bench/2026-11-09_inverter/L2-INV-01_in-phase.png`) |
| .c Frequency | equal (the inverter follows the grid) | — | frequency is the same |
| .d Bulb dark, relay command 0, state 8 held | yes | relay closure inhibited in this stage | relay remains open |
| .e INFO: V_q, omega, time from `c` to state 8 | record | L0-GFL-16: link within one period, after the 1 s qualification and the 1000-call dwell | Held in state 8: Vd_avg 325–329 V, \|Vq_avg\| ≤ 0.22 V, ω_avg 313.8–314.3 rad/s, `grid_ok` 1; state 1 → 8 within one telemetry line (< 0.2 s) with the observer already qualified |

## L2-INV-04 · Step 1b: dry run of the connection, relay power contacts unwired

Step 2 run with nothing behind the relay: the firmware goes through the whole connection sequence and the relay clicks, but its contacts connect nothing. It checks the sequence and measures the relay's real timing before the relay is allowed to connect the inverter to the mains.

**Why the timing matters.** The firmware has no contact feedback: it reports the relay closed 20 ms after the command (`app::relay_contact_ticks` = 200 ticks), and open 20 ms after the command is released. Until it reports the relay closed, the inverter is in GRID_CONNECT: it copies the grid voltage, with no current loop. If the contact really closes earlier, the closing current is set only by the voltage mismatch and the filter (and the bulb). If it closes later, the current loop is already running on an open circuit and can pull the output away from the grid before the contact closes.

**Conditions.** As step 1: bus at 400 V (never at the 100 V of step 0: with the bus below the grid peak, about 325 V, the grid charges the bus through the bridge diodes whenever the relay is closed and the bridge stopped), mains through the bulb, boost IDLE. Both power terminals of the relay disconnected from the circuit and insulated. The grid-side voltage sensor (VAC) stays on the mains, so the firmware still synchronizes. CH1 stays on the filter output (inverter side).

**Scope.** CH4 on GPIO 22 (relay command). CH3 on the relay contact, through a low-voltage circuit: a 5 V supply through 1 kΩ across the contact. CH3 reads 5 V with the contact open and 0 V with it closed. CH3 is not needed for current here: no current can flow, and the telemetry still prints IAC. Trigger: single shot on CH4 rising, 2 ms/div.

**Sequence.**
1. Mains breaker off, DC source off. Disconnect both conductors from the relay's power contacts, insulate the loose ends, wire the 5 V / 1 kΩ circuit and CH3 across the contact. The contact must be connected to nothing else: with one side still on the filter, the 5 V circuit would sit at mains potential.
2. Power up as step 1 (DC source 400 V, then mains). Press `c` and wait for `inverter_state` 8. Check CH1 against CH2 as in step 1.
3. Press `g`. Expected: 10 grid periods (200 ms), then `relay` 10, the click, `relay` 11 20 ms later, `inverter_state` 2 → 3. On the single shot, read:
   - closing time: from the GPIO 22 rising edge to the first fall of CH3;
   - bounce: from the first to the last transition of CH3.
4. Stay in state 3 for 10 s. Compare CH1 with CH2 at about 1 s and at 10 s (RMS and phase, as step 1). The current loop is running with no current to regulate: this shows whether it pulls the output away from the grid.
5. Re-arm the scope on CH4 falling and press `x`. Expected: `inverter_state` 5 → 6 → 0, `relay` 00. Read the release time: from the GPIO 22 falling edge to CH3 back at 5 V (last bounce).
6. Repeat `c`, `g`, `x` twice more and record the closing time, bounce and release time each time.
7. Download the firmware capture (`r`): relay command, state 8 → 2 → 3, IAC about 0.
8. Protection test: `c`, `g` again, and in state 3 press `z`. One control period sees an artificial 3 A on IAC. Expected in the same period: the console prints `inverter bench current trip`, `inverter_state` 7, the bridge stops (CH1 drops to zero), GPIO 22 falls (CH4) and the relay opens. Press `r`: the capture (triggered by the fault) shows the 3 A sample, the mode going to 7 and the duty to 0 on the same sample. Press `x` to clear the latch.
9. Mains breaker off, DC source off. Remove the 5 V circuit and reconnect the relay's power contacts before step 2.

**Acceptance.**

| Criterion | Limit | Why | Result |
|---|---|---|---|
| .a Sequence: state 8 → 2 → 3 held, `relay` 10 → 11, no fault (no PWM_UPDATE_FAILED, no connection timeout) | yes | the connection sequence works before anything is connected | **Pass** after the library fix (#10, `b7fa96c`): 8 → 2 → 3, `relay` 10 → 11, no fault. Before it, fault 12 within 6 ms of `g` |
| .b Closing time + bounce, worst of 3 | ≤ 15 ms (5 ms margin before the 20 ms contact time) | the firmware must not report the relay closed, and start the current loop, before the contact has settled. If it fails, raise `app::relay_contact_ticks` to the worst measured value + 5 ms before step 2 | 9.7 ms command → contact on one operation (`L2-INV-04_relay-close_10ms.png`); worst of 3 still to measure. Since `48d562d` the command leads a voltage zero crossing by 9.7 ms (`app::relay_close_delay`) |
| .c `x`: state 5 → 6 → 0, `relay` 00, no fault; release time, worst of 3 | ≤ 15 ms | the firmware treats the relay as open 20 ms after releasing the command, then stops the bridge | |
| .d Inverter side against grid side after 10 s in state 3 | within ±3 % and ±3°, as step 1 | the current loop with no current must not pull the output away; a drift points to the IAC offset winding up the current integrator, and makes the step 2 closing current depend on the relay timing | |
| .e INFO: the three closing times, bounces and release times; IAC and `Id`, `Iq` in state 3 | record | sets `app::relay_contact_ticks` | |
| .f Protection test (`z`, 3 A for one period in state 3): fault in the same period, bridge stopped, relay command dropped, latched until `x` | yes | checks the bench current trip end to end before the relay connects to the mains | **Pass** (4 captures, e.g. 2026-11-09_21-44-56): on sample 129, IAC 3.00 A, `inverter_mode` 3 → 7, `bridge_duty` 0.71 → 0, `relay_cmd` 1 → 0, all in the same 100 µs period; latched until `x`. A 2 kHz capture misses the single 3 A sample (one period in five recorded) |

## L2-INV-05 · Step 1c: inverter voltage offset, relay open, grid on the other side

**Why.** At the contact the current is driven by the voltage across the open relay: the inverter-side voltage minus the grid voltage. Raising the inverter side a little above the grid (an offset on the feedforward) makes that current flow out of the inverter, with a size set on purpose instead of by chance. The design tests found that the mismatch, not the current loop, drives the connection transient (D-GFL-02), and that the phase matters more than the amplitude (1° is about 5.7 V at the peak, D-GFL-07). The switched model also predicts that the bridge alone already puts the inverter side about 1.1 % and 0.3° above the grid (D-GFL-07): measure it at offset 0 first.

**Firmware.** Keys `+` / `-`: the inverter aims `offset` above the measured grid voltage, as a feedforward gain 1 + offset (`setFeedforwardGain`, library `2f6616f`). Step `app::inverter_vac_offset_step` = 1 % of VAC (about 3.3 V peak) per press, limited to 0–5 % (`app::inverter_vac_offset_max`): never below the grid, at most 16 V peak, which drives at most about 0.4 A peak through the cold bulb (40 Ω) at the contact. The value is printed on each press and in the `sync:` line (`offset`); it is kept across stages until changed or the board restarts. Tested at L0 (`L0-GFL-18`, `L0-APP-10`).

**Conditions.** As step 1, with the grid wired on the other side of the relay through the bulb (as for step 2). Relay open. Scope: CH1 inverter-side L–N, CH2 grid side L–N (both on the same differential-probe scale), or one mains-rated differential probe across the open relay contacts (it shows the mismatch directly). 5 ms/div, trigger on the grid voltage.

**Sequence.**
1. Press `c`; wait for PWM_SETTLE (state 8) and `offset 0.0 %` in the telemetry.
2. At offset 0, record the inverter-side and grid RMS, the phase between them, and the voltage across the relay (peak and RMS).
3. Press `+` once (1 %), wait one second, record the same; repeat up to the offset you want to try (at most 5 %). `-` lowers it, never below 0 %.
4. Choose the offset for step 2: the inverter side clearly above the grid, by a measured amount (1–3 % is 3–10 V peak, 80–250 mA peak out of the inverter through the cold bulb at the contact).
5. Keep it and go to step 2 (`g`).

**Acceptance.**

| Criterion | Limit | Why | Result |
|---|---|---|---|
| .a `offset` follows the keys | 0 → 5 % in 1 % steps; never below 0, never above 5 % | the firmware limits | |
| .b Inverter-side amplitude follows the offset | rises by offset × VAC within ±0.5 % of VAC, phase unchanged within ±1° | the gain scales the feedforward only | |
| .c INFO: voltage across the open relay at each offset (peak, RMS, phase), including offset 0 | record | the voltage that drives the contact current; compare offset 0 with the simulation (+1.1 %, +0.3°) | |
| .d INFO: offset chosen for step 2 | record | — | |

## L2-INV-06 · Step 1d: the bridge copies VAC, relay open (test only)

**Why.** In steps 1 and 1c the inverter-side wave is rebuilt by the controller (SOGI, PLL, feedforward). Here the bridge copies the measured grid voltage directly, so the voltage left across the open relay is what the hardware and the actuation delay alone produce. Comparing it with step 1 separates the controller's share of the mismatch from the hardware's. **The relay is never closed in this stage** (the inverter controller stays OFF and never requests the connection).

**Firmware.** Key `a` (`INV_VAC_FOLLOW`). Once the grid observer has qualified the grid (`grid_ok` 1), the duty is 0.5 + k · (1 + offset) · (VAC extrapolated 1.3 periods ahead − VAC DC estimate) / (2 · VDCBus):
- the extrapolation (`app::vac_follow_advance`) compensates the actuation delay (80 µs to the next carrier period, plus half the 100 µs hold);
- a slow first-order filter (`app::vac_follow_dc_tau`, 2 s) removes the VAC offset (about −5.7 V on the bench); it runs from boot and is settled about 6 s after it (0.5 V of 50 Hz ripple left);
- k ramps from 0 to 1 in 1 s, as in step 0; the offset of step 1c (`+` / `-`) applies;
- the bench current trip stays active. `x` stops it. Tested at L0 (`L0-APP-11`).

**Conditions.** As step 1c: grid wired on the other side of the relay through the bulb, relay open. Same scope setup (CH1 inverter side, CH2 grid side, or a differential probe across the open relay contacts).

**Sequence.**
1. From OFF (`x`), press `a`. Wait for `grid_ok` 1 and one second of ramp.
2. Record the inverter-side and grid RMS, the phase between them and the voltage across the relay (peak, RMS), at offset 0.
3. Optionally, `+` / `-` as in step 1c and record again.
4. Press `c` (step 1) and record the same, to compare with the controller-built wave.
5. Press `x`.

**Acceptance.**

| Criterion | Limit | Why | Result |
|---|---|---|---|
| .a No bridge before `grid_ok` 1; relay never commanded (`relay` 00 throughout) | yes | test-only stage | |
| .b Inverter-side amplitude | within ±2 % of the grid amplitude | the bridge copies VAC | |
| .c Phase between the two sides | within ±1° | the extrapolation compensates the delay | |
| .d INFO: voltage across the open relay (peak, RMS, phase), step 1d against step 1 | record | the controller's share of the mismatch | |

## L2-INV-02 · Step 2: close the relay, then zero current (GRID_CONNECT → GRID_CONNECTED)

**Conditions.** As step 1c: grid wired on the other side of the relay through the cold bulb, inverter in PWM_SETTLE (state 8) with the offset chosen in L2-INV-05 (telemetry `offset`). Scope: trigger on CH4 (relay command) rising, 2 ms/div; CH3 current, CH1 and CH2 voltages.

**What the simulation predicts.** Through the cold bulb the switched model (L1-GFL-09/11) trips the 1.5 A bench protection 20–60 ms after the contact: once connected, VAC is measured on the inverter side of the bulb and the PLL follows the inverter's own voltage. The protection stops it (L2-INV-04.f); a trip here is a result to record, not a failure of the bench.

**Sequence.**
1. Press `g`. The inverter settles 10 grid periods (200 ms), then commands the relay (`relay` 10), and reports it closed 20 ms later (`relay` 11): `inverter_state` 2 → 3 (GRID_CONNECTED).
2. Record the closing transient: peak current on CH3, voltage disturbance on CH1/CH2 around the closing, duration until the current settles.
3. In GRID_CONNECTED, record the residual line current (RMS over 100 ms) and the telemetry `Id`, `Iq` (should be about 0).
4. Download the firmware capture (`r`): relay command, IAC, VAC, state 2 → 3 at 10 kHz.
5. Press `x`: current to zero (state 5), relay opened (state 6, `relay` 00), state 0. Record the opening transient too.

**Acceptance.**

| Criterion | Limit | Why | Result |
|---|---|---|---|
| .a Closing current peak | < 1.5 A, no bench trip | the bench protection level; with ±3 %/±3° matching, about 0.2–0.4 A through the cold bulb | |
| .b Closing transient settled | within 100 ms (5 periods) | the current loop starts in GRID_CONNECT with zero reference | |
| .c Residual current in GRID_CONNECTED | < 0.05 A RMS; bulb dark | the current loop regulates to zero | |
| .d Opening with `x`: relay opens only after the current is at zero; no trip | yes | STOPPING then DISCONNECT | |
| .e INFO: voltage disturbance at closing (peak, duration), relay bounce visible on CH3 | record | — | |
| .f INFO: direction of the current at the contact (out of the inverter with a positive offset) and the offset used | record | step 1c | |

## L2-INV-03 · Step 3: inject a fixed current (RUN)

**Conditions.** As step 2, in GRID_CONNECTED (state 3). The bulb makes a weak grid: the injected current raises the grid-side voltage by I·R_bulb (cold about 40 Ω, hot about 500 Ω). The firmware caps the reference at 0.1 A peak (`app::inject_current_max`), which keeps the rise below the observer's 253 V limit. Higher currents need the bulb bypassed and a new cap: a separate decision.

**Sequence.**
1. Set the reference with `j` (+0.01 A) / `k` (−0.01 A); the telemetry prints `Id_ref`. Start at 0.02 A.
2. Press `n`: `inverter_state` 4 (RUN). Record CH3 (current) and CH2 (grid side) together: current amplitude, phase relative to the voltage, THD; the telemetry `Id`, `Iq`, `P`; the DC source current; `VAC_rms`.
3. Step up to 0.05 A, then 0.1 A, recording each.
4. Press `g` to return to GRID_CONNECTED (current back to zero), then `x`.

**Acceptance.**

| Criterion | Limit | Result |
|---|---|---|
| .a Measured d-axis current (telemetry `Id`, and CH3 fundamental amplitude) | within ±10 % of `Id_ref` (or ±5 mA at the smallest steps) | |
| .b Current in phase with the grid voltage | within ±10°; `Iq` about 0 | |
| .c Grid-side voltage | stays within 207–253 V; no observer trip | |
| .d Power flow direction | DC source current rises with `Id_ref` (power from the bus to the grid) | |
| .e INFO: current THD, `P` against VAC_rms·I_rms, bulb glow | record | |

## Record

Fill one row per step; keep the log and the scope files next to it under `docs/testing/bench/<date>_inverter/` (screenshots named `L2-INV-0x_<what>.png`).

| Date | Step | Firmware commit | VDCBus | Result per criterion | Files | Notes |
|---|---|---|---|---|---|---|
| | L2-INV-10 | | | | | |
| | L2-INV-00 | | | | | |
| 2026-11-09/10 | L2-INV-01 | `c935efb` and later | 399–402 V (meter 399.4 V) | .a pass (RMS very close), .b pass after the leg-order fix, .c pass, .d pass | `2026-11-09_inverter/L2-INV-01_in-phase.png` | bridge leg order and VDCBus calibration fixed during the step |
| 2026-11-09/10 | L2-INV-04 | `b487ba5` (fix), `f352225`/`acf1ad6` (z) | 400–404 V | .a pass, .b 9.7 ms (1 of 3), .c .d to do, .e partial, .f pass | `2026-11-09_inverter/L2-INV-04_*.png`, `src/Data_records/2026-11-09_21-4*` | findings #9 (relay pin), #10 (fault 12), #11 (scope dump) |
| | L2-INV-02 | | | | | |
| | L2-INV-03 | | | | | |

## Simulation counterpart

The same four stages are to be run on the switched model (L1), so each bench result has a simulated reference: same firmware, same commands, an ideal DC source on the bus, the grid through a resistor for the bulb. This needs the L1 PWM adapter to drive the bridge (at 50 kHz, finding #6) and the relay; until then, the references above come from the model tests L1-MOD-04…06.
