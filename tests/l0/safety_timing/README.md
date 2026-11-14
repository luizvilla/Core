# L0-TOV · Safety API timing overhead

Measures what the Safety API costs the control task, before the safety work (Core `main`) and after it (`safety-hw`), on a bare Spin (issue luizvilla/Core#27). The plan, the decisions and every result are in [docs/safety/timing_overhead.md](../../../docs/safety/timing_overhead.md).

```bash
bash tests/common/target_preflight.sh --env=SAFETY_TIME
bash tests/l0/safety_timing/run.sh --run     # four builds flashed in turn, about 10 min; or tests/run_all.sh --l0 --target
```

## Builds

All four builds use the same firmware, [`tests/firmware/safety_timing`](../../firmware/safety_timing/main.cpp), with the Twist 1.4.1 devicetree and the flash at 4 wait states.

| Build | Env | What |
|---|---|---|
| B0 | `TOV_BEFORE` | Core `main` at `8111a0dc`, built in a git worktree by [`build_before.sh`](build_before.sh), plus [`flash_ws4.patch`](flash_ws4.patch) |
| B1 | `SAFETY_TIME` | `safety-hw`, production |
| B1p | `SAFETY_TIME_PROBE` | B1 with `CONFIG_OWNTECH_SAFETY_TIMING_PROBES`: per-block costs of the control interrupt |
| B2 | `SAFETY_TIME_DEV` | development mode, run-time self-tests off, `disarm()` |

## Scenarios and measurements

Each scenario runs in a fresh boot. The first PWM enable arms the protection, and Core `main` cannot create the control task twice.

- **S0:** no safety call.
- **S1:** the old software check on every Twist sensor (`initShield()`).

Both scenarios run at 50 µs and 100 µs. Each control period has a 10 µs busy load, with data dispatch and PWM on. Recorded over 4 s:
- the gap between two task starts: worst cases, histogram in 0.25 µs bins, starts later than 1.5 periods;
- the CPU left to a main-thread loop, which counts the cycles it gets;
- the first output enable (arming).

B1p also records each block of the control interrupt and the HRTIM gate's interrupt-locked window. Every `safety-hw` build records the longest RAM self-test block with interrupts off.

The REACT case (B1, B2) tightens V_HIGH's software minimum to 99 under load and times the stop.

## Bare-Spin adjustments

Floating pins read beyond some Twist limits, so the test overlay widens them:
- I1_LOW, I2_LOW, I_HIGH: low limit −11 A;
- V_HIGH: low limit −10 V;
- V1_LOW, V2_LOW: low limit −150 V (a floating pin reads about −92 V).

The current pins never change, so `CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS` is 100000. The frozen-sensor check still runs at its usual cost, but it never fires within the 4 s window.

## Output

`l0_tov/` holds:
- `timing_tov.md`, the builds side by side;
- `timing_tov.json`, every number and histogram;
- the build logs and the test-case files for the review page.

The limits are provisional ("measure first"):
- safety code per period ≤ 5 % of the period;
- interrupts off ≤ 2 µs;
- jitter ≤ +2 µs against B0;
- CPU ≤ +5 % against B0.

Afterwards the board holds the `SAFETY_TIME_DEV` firmware: flash env USB before a bench session.
