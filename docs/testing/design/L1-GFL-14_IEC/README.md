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

- **Step 2b (stage 6), as it will be flashed:** VAC copy as feedforward until the predicted contact, current PI (Kp 1 V/A, Ti 5 ms) from the relay request, PLL frozen 15 ms from the relay command, synthesized feedforward from the predicted contact to the end of the blanking, GRID_CONNECTED 20 ms after the command, output advance 0.3.
- **Production builds, nothing patched:** the library at the pinned commit (`build` refuses another checkout); the Kp 2 / Ti 20 ms variant only changes two constants in a copy of `src/app/control_step.h`.

| Run | What it checks | Closest averaged result (peak before the step) |
|---|---|---|
| step 2b, IEC grid, 80 µs | the design point | ≈0.4 A (D-GFL-04) |
| step 2b, Kp 2 V/A, Ti 20 ms, IEC, 80 µs | the gain variant | ≈0.3 A (D-GFL-06) |
| step 2b, cold bulb (40 Ω), 80 µs | the bench condition | ≈3.8 A (trips, D-GFL-06) |
| step 2b, IEC, 60 µs | delay sensitivity | ≈7 A (trips, D-GFL-06) |
| step 2b, IEC, 100 µs | delay sensitivity | ≈7 A (trips, D-GFL-06) |
| step 2 (today), IEC, 60 µs | today's connection at the other plausible delay | 9.8 A (trips) |

The averaged results are the closest simulated cases: D-GFL-04/06 used the synthesized feedforward from the relay command, while step 2b copies VAC until the contact (not simulated). The delay rows matter most: the averaged model says the result depends on the actuation delay to within 20 µs, because the filter resonance with the relay closed sits at the 10 kHz control rate (D-GFL-07).
