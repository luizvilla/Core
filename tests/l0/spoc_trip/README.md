# L0-TRP · protection paths trip at their thresholds

Drives every limit, bound and path of the board's protection manifest across its threshold on the CUT Spin of the SPOC bench, and checks the trip (ADR 0009, issue luizvilla/Core#23, C11). The cases come from the manifest, so the same runner serves any board with a test env.

```bash
bash tests/common/spoc_preflight.sh                           # CUT, MUX and scope answer
bash tests/l0/spoc_cal/run.sh --run                           # calibrate the bench first (L0-CAL)
bash tests/l0/spoc_trip/run.sh --run                          # flashes SAFETY_L0_TWIST, runs every case
bash tests/l0/spoc_trip/run.sh --run --board=ownverter        # SAFETY_L0_OWNVERTER (Ownverter 1.1.0)
.venv/bin/python tests/l0/spoc_trip/run.py --out /tmp/x --no-flash   # development run, not a campaign
```

The CUT keeps the test build after the run: it stays on the safety test firmware between campaigns.

## How a case runs

| Step | What |
|---|---|
| Build | `SAFETY_L0_TWIST`: the production allocation of Twist 1.4.2 plus the reserved injection DAC (DAC2_CH1 on PA6). L0-TRP-01.a compares its protection manifest with `tools/safety_check.py`'s production one |
| Reset | A fresh boot: the arming self-tests (watchdog windows, idle plausibility of the current sensors) run once, at the first data start |
| First arming | Every current pin whose low bound is an ADC watchdog held at its 0 A level; PWM on, quiet, PWM off (L0-TRP-01.b). The trips then stay armed until the next reset |
| Routes | With the PWM stopped: the MUX injector holds the other current pins through CH3 and, with TP5-TP7 wired, CH4; CH1 to PA6 and CH2 to the pin under test (TP1-TP3 joins them) |
| Start | The CUT DAC at 150 mV inside the threshold, only the sensor under test watched by the software check (`/Sw`), clear, PWM on, 200 ms quiet |
| Coarse ramp | 10 mV steps, 30 ms each, until the trip |
| Fine ramp | Recover (PWM off, start level, clear, PWM on), then 1 mV steps from 15 mV before the coarse trip |
| Verdict | Trip fault; for a hardware path its own FLT set, for a software path the software trip of that sensor; outputs off; fine trip level within ±25 mV of the threshold |

The threshold comes from the manifest's counts on the CUT's VREFBUF (2048 mV), which the DAC, the ADCs and the comparator DACs share, so their reference errors cancel. An 8-bit watchdog compares `raw >> 4`: it trips at `(HT + 1) × 16` counts or below `LT × 16`.

## Wiring and BLOCKED cases

A floating current pin reads about -4 A and fails the arming self-tests, so the current pins with an ADC-watchdog low bound must be held while any other pin is driven. Twist 1.4.2 has two (I1_LOW on PA1, I2_LOW on PC1); so has Ownverter 1.1.0 (I1_LOW, I3_LOW).

- Without TP5-TP7: the injector holds one pin (CH3). Only the cases on those two current limits run; the others report BLOCKED.
- With TP5-TP7 wired: the injector holds two pins (CH3 and CH4) and every case runs. CH4 then cannot watch a PWM pin; latency checks use a direct probe.

The runner finds the wire itself: with every route off and the injector at 1.2 V, scope CH4 reads 1.2 V only through the wire.

## Criteria

| ID | What | Limit |
|---|---|---|
| L0-TRP-01.<board>.a | test build manifest against production | identical but for reserved DAC2_CH1 |
| L0-TRP-01.<board>.b | first arming with the held pins at 0 A | armed, outputs on, quiet |
| L0-TRP-02.<board>.<sensor>.<bound> | hardware path (comparator or ADC watchdog) | its FLT, outputs off, ±25 mV |
| L0-TRP-03.<board>.<sensor>.<bound> | software path | software trip of the sensor, outputs off, ±25 mV |

The start level is 150 mV inside the threshold, or 40 % of the sensor's window if that is narrower (Ownverter 1.1.0's I3_LOW window is 157 mV wide, see spoc_bench.md). Held pins with 0 A levels less than 50 mV apart share the injector's mean level (Ownverter: I1 1000 mV, I3 1024.5 mV).

Not covered here: trip latency (C13), glitch filters and edge cases (C14), CPU-dead cases (C12).

History and findings: [docs/safety/spoc_bench.md](../../../docs/safety/spoc_bench.md).
