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

L1 runs the real application through the bench stages on the switched model, with the candidates built from patched library copies ([`variant.py`](../../tests/l1/design/variant.py)). The harness extension is contract app-v3: the C-Block drives the bridge and the relay. Results: to be added (L1-GFL-09…13).
