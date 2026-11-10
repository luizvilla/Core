# L1-GFL-14: how to run

Six switched-model runs that validate the connection strategy found by the design tests D-GFL-04 and D-GFL-06 (averaged model, ADR 0004). About 30 minutes on a machine that runs all six in parallel.

```bash
git pull
python3 tests/l1/design/connection_l1.py build          # study builds, about a minute
python3 tests/l1/design/connection_l1.py run            # six runs in parallel (--jobs N to limit)
python3 tests/l1/design/connection_l1.py report         # writes report.md in this folder
python3 tests/common/control_center.py                  # L1-GFL-14 then links the report
git add docs/testing/design/L1-GFL-14_IEC/report.md && git commit -m "test(l1): L1-GFL-14 results" && git push
```

The run outputs stay in `tests/out/connection-l1/` (local, not committed). Only `report.md` is committed.

## What is being tested

- **Strategy:** the current PI runs from the relay request, the output advance is 0.3 period, and from the relay command until 5 ms after the contact the feedforward is the period-averaged Vd rotated by the PLL angle while the PLL frequency is frozen.
- **Study builds, nothing changes in production:** `connection_l1.py build` patches copies of the library (`holdConnectionReference()`) and of `src/app/control_step.{h,cpp}` under `tests/out/connection-l1/variants/`.

| Run | What it checks | Averaged prediction (peak before the step) |
|---|---|---|
| best, IEC grid, 80 µs | the design point | 0.37–0.41 A |
| best, Kp 2 V/A, Ti 20 ms, IEC, 80 µs | the gain variant | 0.25–0.32 A |
| best, cold bulb (40 Ω), 80 µs | the bench condition | 3.8 A (trips) |
| best, IEC, 60 µs | delay sensitivity | 7.0 A (trips) |
| best, IEC, 100 µs | delay sensitivity | 7.3 A (trips) |
| today, IEC, 60 µs | today's settings at the other plausible delay | 9.8 A (trips) |

The delay rows matter most: the averaged model says the result depends on the actuation delay to within 20 µs, because the filter resonance with the relay closed sits at the 10 kHz control rate (D-GFL-07). If the switched model agrees, the next design step is that resonance, not the connection strategy.
