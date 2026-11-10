# Inverter current loop: analytic design

Status: draft, 2026-11-10. No production change yet; the gains are a decision for Luiz Villa (library `single_phase_inverter`, then a PR to Jean Alinei). Tests: L1-GFL-09…13 ([catalogue](../../tests/catalogue.json)).

## 1. What the loop is today

In the FOLLOWING inverter (`singlePhaseInverterFollowing.cpp`), the bridge voltage reference is

`V_dq,out = V_dq (grid-voltage feedforward) + PI(I_dq,ref − I_dq)`,

rotated back with the PLL angle and turned into a duty: `D = v_α / (2·V_bus) + 0.5`.

| Element | Value today | Source |
|---|---|---|
| Current PI | Kp = 0.001 V/A, Ti = 0.3 ms, output ±30 V | hard-coded in `init()`, no derivation; the library README lists it as "to tune" |
| Current measurement | SOGI on the line current, Kr = 500 rad/s (shared with the voltage SOGI), then park | `init()` |
| Feedforward | SOGI of the PCC voltage (VAC, grid side of the relay), Kr = 500 rad/s | `init()` |
| Output filter | 500 µH (2 × 250 µH), **no output chokes** on the bench board | Luiz Villa, 2026-11-10; model `AC_L1`, `AC_L3` |
| Control period | 100 µs | `app::control_task_period` |
| Bridge | unipolar, 50 kHz; duty limited to 0.1–0.9 by the driver's default user limits, 0.05–0.95 possible with the shield functions | `hrtim.c`, `Power.cpp` `setInverterDutyCycle` |

Kp = 0.001 V/A on 500 µH puts the crossover near 0.3 Hz and the integral gain (Kp/Ti) at 3.3 V per A·s. **There is no current loop in practice: the inverter is a voltage follower.** Nothing validated the current loop before this work (L0-GFL only checks mode transitions; the L1 injection cases were blocked).

## 2. Method

[`tests/l1/design/current_loop.py`](../../tests/l1/design/current_loop.py): an averaged discrete-time model of the loop exactly as the library computes it.
- **Library arithmetic copied:** the SOGI recursion (`sogi.cpp`, float32), the park transforms, and the PI of `control_library` (`pid.cpp`: standard form, integral recomputed on saturation).
- **Actuation:** duty limits; the command takes effect one control period later.
- **Plant:** filter (500 µH, 0.1 Ω assumed) and grid impedance, integrated exactly in 10 sub-steps.
- **Sequence:** relay open with the feedforward running (PIs reset), then relay closed with I_d,ref = 0, then a step.
- **Linear margins:** loop C·G·F·e^(−s·1.5Ts), with F = (Kr/2)/(s + Kr/2) the dq envelope of the current SOGI.

Grids:
- stiff (no impedance);
- **IEC 60725 reference** (0.4 Ω + 0.8 mH, phase plus neutral);
- the bench bulb, 40 Ω cold and 500 Ω hot.

## 3. Findings

1. **The current SOGI limits the bandwidth.** In the dq frame it acts as a first-order low-pass at Kr/2: 250 rad/s (40 Hz) with Kr = 500. With that filter in the feedback path, Kp cannot go above about 0.05 V/A with a usable phase margin. Raising the **current** SOGI gain to 2000 rad/s (the voltage SOGI can stay at 500) allows Kp of 1–3 V/A.
2. **The feedforward phase matters on a low-impedance grid.** 500 µH is 0.16 Ω at 50 Hz, so 1 V of feedforward error drives about 6 A through a stiff grid. The library's SOGI output leads its input by about one sample (1.8°), and the actuation lags 1–1.5 samples. With today's gains this leaves 9–27 A peak on the stiff and reference grids. With a real PI (candidates C, D) the integral removes the error: the steady error before the step is a few mA, and the result barely depends on a delay compensation of 0 to 1 period (§4, sensitivity).
3. **On the bench bulb, the PCC feedforward cancels the bulb resistance.** VAC is measured between the relay and the bulb, so the feedforward contains the bulb's own drop R·i. The current then sees no resistance at low frequency: the plant behaves as an integrator with an equivalent inductance of L + R/(Kr_v/2) (0.16 H cold, 2 H hot). The PI alone holds the current.
   - With today's gains, the current drifts: **7.5 A peak through the cold bulb in the model.** Step 2 on the bench (`g`) would likely have tripped the 1.5 A bench protection.
   - With candidate C or D it stays at 0.15–0.5 A peak and settles in 0.25–0.8 s. That is acceptable for a bench test.
4. **The driver's default duty limits are too narrow at 400 V.** 0.1–0.9 gives ±320 V against a 325 V grid peak (358 V at 253 V RMS): the duty clips in 11–18 % of the periods, with amps of distortion current. With **0.05–0.95** (±360 V) it does not clip at nominal voltage. At the upper grid limit (358 V) the margin is 2 V, so a 420–450 V bus is worth considering.

## 4. Candidates (averaged model, 400 V, duty 0.05–0.95)

| Candidate | Kr_i (rad/s) | Kp (V/A) | Ti (ms) | Delay comp. (periods) | IEC grid: PM (°) | IEC: overshoot (%) | IEC: settle 5 % (ms) | Stiff: PM (°) | Cold bulb: settle 5 % (ms) |
|---|---|---|---|---|---|---|---|---|---|
| A: library today | 500 | 0.001 | 0.3 | 0 | — (no loop) | — | never | — | never (7.5 A drift) |
| **C** | 2000 | 1 | 5 | 0.5 | 69 | 12–14 | 19 | 28 | 812 |
| D | 2000 | 3 | 20 | 0.5 | 39 | 5–6 | 7 | 6.6 | 259 |

- **D** is faster, but its margins are small on a stiff grid (PM 6.6°, GM 2.4 dB).
- **C** keeps 28° on a stiff grid and 69° on the reference grid.
- **Recommendation pending L1:** C, or a Kp between 1 and 2 V/A with Ti 5–10 ms.

Sensitivity to the delay compensation, IEC grid, 2.77 A step: from 0 to 1 period, C's overshoot changes by less than 1 point and D's goes from 10 % to 2 %. The compensation helps a little and is not critical once the PI is real.

Full tables: run the script (about 5 minutes).

## 5. Library changes this implies (for the PR, not done)

- Current PI gains settable from the application (as for the boost), or new defaults.
- A current SOGI gain separate from the voltage SOGI gain.
- Optional: an output rotation advanced by a fraction of the control period (delay compensation).
- In the application: duty limits of the inverter legs set to 0.05–0.95 (`setDutyCycleMin/Max`), and possibly a higher bus voltage.

## 6. L1 verification on the switched model

L1 runs the real application through the bench stages on the switched model, with the candidates built from patched library copies ([`variant.py`](../../tests/l1/design/variant.py)). The harness extension is contract app-v3: the C-Block drives the bridge and the relay.

### 6.1 First batch (2026-11-10): every candidate trips at the connection

Scenarios [`bench_bulb`](../../tests/l1/scenarios/inverter/bench_bulb.json) and [`bench_iec`](../../tests/l1/scenarios/inverter/bench_iec.json). Relay commanded at 1.500 s, contact 9.7 ms later at a voltage zero crossing (within 1.8°), feedback 20 ms after the command.

| Run | Fault | Closing peak (A) |
|---|---|---|
| A, cold bulb | 1.518 s, code 12 (sync lost) | 0.82 |
| A, IEC grid | 1.510 s, bench trip | 1.52 |
| C, cold bulb | 1.560 s, bench trip | 1.62 |
| C, IEC grid | 1.516 s, bench trip | 1.63 |
| D, cold bulb | 1.560 s, bench trip | 1.62 |
| D, IEC grid | 1.516 s, bench trip | 1.63 |

None reached the injection step. Two mechanisms, neither one about the gains:

1. **No current loop between the contact and the feedback.** The library runs the current PI only in GRID_CONNECTED, RUN and STOPPING. In GRID_CONNECT the output is the voltage feedforward alone, and the application reports the relay closed 20 ms after the command, about 10 ms after the contact. On the IEC grid the feedforward error drives the current to the 1.5 A trip within 6 ms of the contact, at the voltage peak. The trip is identical for C and D: their PIs never ran.
2. **Through the bulb, the PLL locks on the inverter's own voltage.** VAC is measured between the relay and the bulb. After the contact it is the inverter's voltage plus R·i, so the PLL and the feedforward both follow the inverter itself: Vq swings ±15 V and the PLL frequency runs to 333 rad/s (53 Hz). With today's gains that ends in fault 12 within 8 ms. With C and D the current grows to the trip about 40 ms after the feedback. The averaged model (§2) used an ideal PLL angle, so it did not show this.

### 6.2 Second batch (running): current PI from GRID_CONNECT

Variant F: the library with the setters (`ab35e2f`), candidate C set from the application, and the current PI also active in GRID_CONNECT with a zero reference. With the relay open the measured current is zero, so the PI stays at zero until the contact. Runs: cold bulb, hot bulb (500 Ω), IEC grid, stiff grid. Variant E (the same library without the GRID_CONNECT change, C from the application) on the IEC grid checks that the application path reproduces candidate C.

| Run | Fault | Closing peak (A) |
|---|---|---|
| E, IEC grid | 1.5156 s, bench trip | 1.63 (identical to C: the application path reproduces the candidate) |
| F, IEC grid | 1.5159 s, bench trip | 1.58 |
| F, stiff grid | 1.5181 s, bench trip | 12.7 |
| F, cold bulb | 1.5591 s, bench trip | 1.62 |
| F, hot bulb (500 Ω) | 1.6561 s, code 1 (BAD_GRID_QUALITY, observer) | 0.54; reached RUN, final I_d 0.044 A for 0.1 A, THD 36 % |

**Running the PI from the contact does not help.** The current oscillates at about 2 kHz from the contact on. The cause is the current measurement, not the gains:
- The library's SOGI computes β by a finite difference scaled by 1/(ω·Ts) ≈ 32. Any fast change of the sampled current is amplified in β: the contact step, the capacitor currents of the output filter, the switching ripple.
- In the 5 ms after the contact, |I_dq| as the controller sees it reaches 3.2 A while the actual line current stays below 0.95 A (IEC and stiff grids; 0.43 A against 0.20 A through the bulb).
- With Kp = 1 V/A, 3 A of measurement error is 3 V of command error each period. The loop closes on its own measurement noise and diverges.

The averaged model (§2) copied the SOGI recursion but fed it a smooth current (no filter capacitors, no ripple, no contact step), so it could not show this. The same β construction was already behind finding #7 (harmonics amplified on the voltage).

**Consequence:** no PI gain fixes this while the dq current comes from this SOGI. Options:
- (a) a SOGI whose quadrature output is integrated (the standard qv = ω·∫ε form, a low-pass) instead of differenced;
- (b) a low-pass or ripple-synchronous filter on IAC before the SOGI;
- (c) a much lower Kp, which brings back the slow drift through the bulb.

Library decision, then L1 again.
