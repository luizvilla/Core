# L0-LAT · trip latency and faults during dead time (direct probes)

This test measures, on the CUT pins, how fast the hardware trips turn the PWM outputs off, and what happens when a fault lands in the dead time. It is part of ADR 0009 and issue luizvilla/Core#23: C13, plus the open part of C14.

The SPOC mux path adds tens of ns, so this test uses passive probes on the CUT itself.

```bash
bash tests/l0/spoc_latency/run.sh --run          # campaign of its own (Twist 1.4.2), about 15 min
```

## Wiring

L0-TRP's wiring (TP1–TP3 joined; the injector on TP5) stays in place. Three scope inputs move from the SPOC BNCs to probes:

| Scope | Connection |
|---|---|
| CH1 | ×10 passive probe on CUT PA8 (Spin pin 12, leg 1 high side) |
| CH2 | ×10 passive probe on CUT PA9 (pin 14, leg 1 low side) |
| CH3 | SPOC BNC3, as before: the injector node TP5, for the record |
| CH4 | ×10 passive probe on CUT PC1 (pin 25, I2_LOW, the COMP3 input) |

The probes are not deskewed. They are the same model, so the skew between them is under 1 ns, against limits of 360 ns and 16 µs.

If the 200 kHz PWM does not show on CH1 and CH2, every case reports BLOCKED. This happens, for example, when the BNC cables are still on CH1 and CH2.

## How it runs

- **Firmware:** `SAFETY_L0_TWIST`. The legs are set up as shield.power sets them, including the devicetree dead time of 100/100 ns. `/Pwm rDtR` reads DTxR back.
- **Holds:** the CUT DAC holds I1 (PA1) at 0 A through CH1 → TP1–TP3 → CH2. The MUX injector's DAC source holds I2 (PC1) at 0 A through CH3.
- **Each trip:**
  1. Arm the PWM: 200 kHz, duty 0.5, up-down counting.
  2. Arm a single capture on CH4 crossing the threshold.
  3. Step the injector to 2000 mV (high bound, 1700 mV) or to 100 mV (low bound, 296 mV).
  4. Read the fault.
  5. Restore the hold, stop the PWM, wait out the 100 ms repeat-clear window, and clear.
- **Timing:** the trips are asynchronous to the PWM, since the MUX is a separate MCU, so the fault instant spreads over the period.
- **Turn-off:** the turn-off of a trip is the last moment either output is above 0.5 V after the crossing. Usually the fault forces it, and turn-off minus crossing is the latency. The level is 0.5 V and not 1.65 V so that an output that starts to turn on counts: when the fault lands just as a dead time ends, the next output rises for one HRTIM clock (about 5 ns, peaks up to 1.64 V seen) before the fault cuts it, and that cut is the turn-off.
- **Dead-time trips:** when the fault lands in a dead time, the last turn-off is the PWM's own edge before the fault, and the output due next never turns on. These turn-offs come early.
  - High-bound turn-offs more than 40 ns below the median are counted as dead-time trips.
  - The others are latencies.
  - The low-bound path converts I2 once every three triggers (ADC2 converts one channel of its three per trigger), so its latency spreads over three periods; its trips are not sorted this way.
- **Odd trips:** the runner keeps the waveforms of the first two trips and of every trip with an early turn-off, a residual peak above 0.2 V, an overlap, or no turn-off before the record ends.

## Criteria

| ID | What | Limit |
|---|---|---|
| L0-LAT-01 | dead time of leg 1 on PA8/PA9 | 100 ns ±25 ns, both edges |
| L0-LAT-02 | I2_LOW high bound, COMP3 → FLT5: crossing to turn-off | ≤ 360 ns (31 ns comparator, DS12288 Table 60, + 282 ns filter + 47 ns filter sample) on ≥ 10 trips; FLT5 on each |
| L0-LAT-03 | I2_LOW low bound, ADC2 watchdog → FLT6: crossing to turn-off | ≤ (channels on ADC2) × period + one conversion (ADR 0009; Twist: V2_LOW, I2_LOW, I_HIGH, 3 × 5 µs + 1 µs = 16 µs) on ≥ 10 trips, each turned off within the 20 µs record; FLT6 on each |
| L0-LAT-04 | every trip | PA8 and PA9 never both above 1.65 V; both below 0.5 V from 30 ns (or three samples) after the turn-off |
| L0-LAT-05 | high-bound trips in the dead time | ≥ 3; the output due next never turns on |

The 31 ns datasheet figure covers the comparator alone. The production paths run through the 282 ns fault filter (fHRTIM/8, N = 6), so the measured time is mostly the filter. L0-EDG-06 measures where the filter cuts.
