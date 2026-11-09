# Inverter bench session

The first hardware tests of the inverter: generate 230 V RMS, synchronize with the grid, close the relay, inject a small current. Each step has its own firmware stage, selected from the serial menu, and its own record. The catalogue IDs are `L2-INV-00…03` (Inverter tab, level L2: on the board); the pre-checks are `L2-INV-10`.

| Step | Menu | Firmware stage | Inverter controller state | What it shows |
|---|---|---|---|---|
| 0 | `o` | `INV_OPEN_LOOP` | not used (OFF) | The bridge and filter make a 230 V RMS, 50 Hz wave with no grid |
| 1 | `c` | `INV_SETTLE` | PWM_SETTLE, held | The inverter-side voltage matches the grid before the relay |
| 2 | `g` | `INV_CONNECT` | PWM_SETTLE (10 periods) → GRID_CONNECT → GRID_CONNECTED | The relay closing disturbance, then zero current on the grid |
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

Every 100 ms the board prints one line, fields separated by `:`:

`ILow1 : VLow : duty1 : VAC : VAC_rms : grid_angle : IAC : VDCBus : boost_mode : burst_skip : stage : inverter_state : relay(cmd)(fb) : Id : Iq : P : Q : Id_ref : load_us`

- `stage`: 0 OFF, 1 SETTLE, 2 OPEN_LOOP, 3 CONNECT, 4 INJECT.
- `inverter_state`: 0 OFF, 1 VOLTAGE_SYNC, 2 GRID_CONNECT, 3 GRID_CONNECTED, 4 RUN, 5 STOPPING, 6 DISCONNECT, 7 FAULT, 8 PWM_SETTLE, 9 PWM_WAIT.
- `relay`: two digits, command then feedback (feedback = command held 20 ms).
- A `sync:` line: the synchronization criterion as the inverter judges it (one-period averages, finding #7): `Vd_avg` 293–358 V, `|Vq_avg|` < 3.25 V, `w_avg` 298.5–329.9 rad/s, `window` 1 once full, `grid_ok` (grid observer verdict) and `trip` (observer trip reason: 0 none, 1 invalid measurement, 2 UV, 3 OV, 4 UF, 5 OF). **A tripped observer stays tripped until the board restarts** (finding #2): power-cycle after any trip before retrying a grid step.
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
| .a RMS difference, inverter side − grid side | within ±3 % (about ±7 V) | Sets the relay closing current: about ΔV / R_bulb (cold) = 7 V / 40 Ω ≈ 0.2 A | |
| .b Phase difference | within ±3° (about 0.17 ms) | 3° of 325 V is 17 V of instantaneous difference | |
| .c Frequency | equal (the inverter follows the grid) | — | |
| .d Bulb dark, relay command 0, state 8 held | yes | relay closure inhibited in this stage | |
| .e INFO: V_q, omega, time from `c` to state 8 | record | L0-GFL-16: link within one period, after the 1 s qualification and the 1000-call dwell | |

## L2-INV-02 · Step 2: close the relay, then zero current (GRID_CONNECT → GRID_CONNECTED)

**Conditions.** As step 1, inverter in PWM_SETTLE (state 8). Scope: trigger on CH4 (relay command) rising, 2 ms/div; CH3 current, CH1 and CH2 voltages.

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
| | L2-INV-01 | | | | | |
| | L2-INV-02 | | | | | |
| | L2-INV-03 | | | | | |

## Simulation counterpart

The same four stages are to be run on the switched model (L1), so each bench result has a simulated reference: same firmware, same commands, an ideal DC source on the bus, the grid through a resistor for the bulb. This needs the L1 PWM adapter to drive the bridge (at 50 kHz, finding #6) and the relay; until then, the references above come from the model tests L1-MOD-04…06.
