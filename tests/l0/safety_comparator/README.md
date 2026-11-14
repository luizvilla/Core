# L0-CMP / L0-AWD / L0-SST · Hardware trips and safe states on the Spin

Checks the comparator trips of [ADR 0009](../../../docs/adr/0009-safety-api-hardware-protection.md) (issue luizvilla/Core#18): each comparator trip the build allocated is armed, self-tested and locked before the first PWM output, a sensor beyond its limit turns every PWM output off in hardware, the locks hold, a threshold changed behind the API trips, and a trip stays latched.

```bash
bash tests/common/target_preflight.sh --env=SAFETY_COMP
bash tests/l0/safety_comparator/run.sh --run     # build, flash, run (about 30 s); or tests/run_all.sh --l0 --target
```

Hardware: a bare Spin and an ST-Link, no wiring, no scope. The firmware ([`tests/firmware/safety_comparator`](../../firmware/safety_comparator/main.cpp), env `SAFETY_COMP`) uses the Twist 1.4.1 devicetree: I1_LOW on PA1 (COMP1, DAC3_CH1 → FLT4) and I2_LOW on PC1 (COMP3, DAC1_CH1 → FLT5), limit 7 A = DAC code 3400. With no shield the sensor pins float near 0 V, below the limits. A trip is provoked by **stepping the sensor pin to 3.3 V from the test firmware** (GPIO output); the time is the cycle count from the step to the HRTIM fault flag, with the Safety API's fault interrupt masked only while timing. Production code never drives a sensor pin.

The protocol (command word over SWD, acknowledgement in RAM, pending mark in backup registers for the case that resets) is the one of [L0-SUP](../safety_supervision/README.md), with the shared helpers in [`tests/common/target_harness.py`](../../common/target_harness.py).

## Findings while building it (2026-11-14)

- **The HRTIM fault input ignores the comparator's POL bit**: it receives the output before the polarity selector. Low-bound trips use an active-low fault input instead; the arming self-test flips the fault input's polarity, not the comparator's.
- **A failed first arming left the comparators enabled**, and the retry then mistook them for peak-current mode. The module now remembers which comparators it configured and disables them after a failed attempt.
- Step to fault flag: **0.476 µs** (declared ≤ 0.5 µs; the 282 ns filter is most of it).

## ADC-watchdog path (L0-AWD, issue luizvilla/Core#19)

The same firmware arms the Twist 1.4.1 watchdogs (I1_LOW ADC1 AWD1, V_HIGH ADC1 AWD2, V1_LOW ADC1 AWD3, I2_LOW ADC2 AWD3) on data acquisition, which the Safety API starts itself here (no application data acquisition). A test overlay lowers the current low limits to −11 A so floating pins sit inside the windows. A pin cannot hold a level against the ADC sampling capacitor, so the watchdog trip is provoked by the window: ADC1 stopped, a window every reading falls out of, ADC1 restarted on its HRTIM trigger (test code only).

Findings: the first allocation watched channels on the wrong ADC (now: the sensor's default ADC only); the ADC driver kept a software trigger after the hook changed the register (now changed through DataAPI); a DMA interrupt per sequence starved the CPU (now `startUnattended`, no interrupt); FLT6 is bit 6 of the HRTIM flags. Measured: comparator output to fault flag 0.347 µs; ADC restart to fault flag 1.72 µs.

## Safe states (L0-SST, issue luizvilla/Core#20)

The test overlay sets LEG2 (timer C) `safe-state = "short-low"` and declares a safe output on Spin pin 21 (PB9); `app.conf` shortens the repeat-clear window to 100 ms so back-to-back cases are not refused. Checked: fault levels in OUTxR, PWM pins after each trip against them (TC2 on, all else off), the test relay dropping, the read-back fault stopping through a forced hardware fault (external event), the fault text, and the repeat-clear rule (runs last: it leaves the board needing a reboot).

Not covered here: the low-bound comparator in service (the self-test exercises the active-low path on every comparator; an in-service check needs a stimulus that holds a level: SPOC bench).
