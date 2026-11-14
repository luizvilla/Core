# L0-CMP · Comparator path on the Spin

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

Not covered here: the low-bound comparator in service (the self-test exercises the active-low path on every comparator; an in-service check needs a stimulus that holds a level: SPOC bench).
