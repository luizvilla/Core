---
status: accepted
date: 2026-11-13
---

# IAC sampled twice per switching period, half a period apart

Decided by Luiz Villa on 2026-11-13, with Claude, after the bench sessions of L2-INV-12 and L2-INV-13 (branch `apf_quadrature`).

## Context

With the relay closed the firmware IAC read about +145 mA of DC that the line did not carry: the DC current loop (key `B`, L2-INV-12) drove the firmware reading to its learnt zero and the DC ammeter to −140 mA. With the relay open the reading was right.

ADC_2 (IAC, ILow1) was triggered once per switching period by CMP3 of the LEG2_HIGH timer (PWME, 50 kHz, up-down) at 0, the carrier valley, on the up-count only (`default-edge-trigger = "EdgeTrigger_up"`, ADROM = 10). One sample per period aliases every harmonic of the switching frequency to DC.

The line current carries the switching ripple (MSO54 on CH3, session `tests/out/2026-11-13_c862597_222057_bench`, connected behind the bulb, filter on): 344 mA peak at 50 kHz, 52 mA at 100 kHz, 75 mA at 150 kHz, 11 and 21 mA at 250 and 350 kHz, 2.4, 1.6 and 2.8 mA at 200, 300 and 400 kHz. Resampling that record at the switching frequency from every phase gives the DC error of each sampling scheme:

| Sampling | DC error over all phases |
|---|---|
| 1 sample per period | −544 to +534 mA |
| 2 samples T/2 apart | −8 to +11 mA |
| 4 samples T/4 apart | −5 to +7 mA |

With one sample, the error crosses zero twice per period on a steep slope: ±20 mA means hitting the instant within about ±120 ns. The ripple phase depends on the grid impedance (the share of the inductor ripple that leaves through the relay rather than the filter capacitor), on the duty over the grid cycle and on the dead time, so no fixed instant holds.

## Decision

- LEG2_HIGH triggers ADC_2 on CMP3 **while counting up and down** (overlay `default-edge-trigger = "EdgeTrigger_Both"`, ADROM = 00; RM0440 §27.5, ADC post-scaler: ADROM selects the counting phases of every ADC trigger event, compare events included).
- CMP3 is set to **0.5 of the period at start-up**: the two samples fall at T/4 and 3T/4, half a period apart. The 50 kHz ripple and its odd harmonics cancel in their mean whatever their phase and amplitude, so the reading no longer depends on the grid impedance. The even harmonics remain (small here, and the replay shows they average out over the grid cycle).
- Keys `[` / `]` still move CMP3 (0 to 0.95) for checking; away from 0.5 the samples are no longer T/2 apart.

## Consequences

- ADC_2 converts twice as often: the control step averages 4 to 6 IAC samples per period instead of 2 or 3 (ADR 0007). The DMA buffer of ADC_2 holds 21 sequences (210 µs), well above one period.
- ILow1 shares the trigger. The boost leg runs at 200 kHz, centre-aligned with no phase shift: T/4 and 3T/4 of the bridge period (5 and 15 µs) are boost valleys or crests, where the boost inductor current equals its mean, as long as both counters are aligned (assumed from the phase-0 configuration, not checked on the scope).
- An overlay change: PlatformIO does not rebuild on overlay edits, so a clean build is needed; check `default-edge-trigger` in `.pio/build/USB/zephyr/zephyr.dts`.
- Four samples (T/4 apart) would also remove the even harmonics but need a second trigger source; not needed while the even harmonics stay small. An analog low-pass on the IAC sensor would be the most robust option, but it is a hardware change.
