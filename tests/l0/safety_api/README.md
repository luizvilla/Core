# L0-API · One Safety API on the Spin

Checks the "one Safety API" part of [ADR 0009](../../../docs/adr/0009-safety-api-hardware-protection.md) (issue luizvilla/Core#22):
- the legacy functions are compatibility wrappers of the protection;
- the old software check reacts like every other trip;
- development mode is the only way to disarm or loosen.

```bash
bash tests/common/target_preflight.sh --env=SAFETY_API
bash tests/l0/safety_api/run.sh --run     # build test + two builds flashed and run (about 6 min); or tests/run_all.sh --l0 --target
```

## What runs

- **L0-API-01, build test, no board** ([`build_mode.py`](build_mode.py), env `SAFETY_MODE`):
  - `safety.disarm()` in a production build must fail to compile, and the error must name `safety-mode`;
  - the same source must build with `safety-mode = "development"`;
  - run-time self-tests turned off in `app.conf` without development mode must fail to build.
- **L0-API-02..06, on target** ([`run.py`](run.py)): one firmware ([`tests/firmware/safety_api`](../../firmware/safety_api/main.cpp)) built twice:
  - env `SAFETY_API`: production;
  - env `SAFETY_API_DEV`: the same sources and devicetree, plus a `safety-config` node in development mode.
  Each build is flashed in turn and driven over SWD with the protocol of [L0-SUP](../safety_supervision/README.md).

Hardware: a bare Spin and an ST-Link, no wiring. The Twist 1.4.1 devicetree is used:
- V_HIGH limits 10..100 V; V1_LOW and V2_LOW −10..70 V;
- the current low limits are lowered to −11 A, so the floating pins sit inside the ADC-watchdog windows;
- a safe output on PB9 stands for a relay.

The software trip uses V_HIGH, which reads about 0 V on a bare Spin. Its software minimum is tightened to 99.

**NVS:** the test writes the V_HIGH threshold entry (0x0303). Afterwards it puts back the board's own entry, or deletes the test entry if there was none (new `nvs_storage_delete_data()`). L0-API-04.c checks this.

## Findings while building it (2026-11-14)

- **The software check could latch while the PWM was starting and the outputs were enabled anyway.**
  - The first `startDualOutput()` arms the hardware trips, which takes milliseconds.
  - During that time the software check (control-task interrupt) latched a trip and stopped every output.
  - The HRTIM driver then enabled the unit's outputs, because the gate had already answered yes. The first run measured a fault latched with TA2 switching.
  - Fix: `hrtim_out_en()` asks the gate again with interrupts locked before it writes the enable. L0-API-05.h and 06.g.h keep this covered.
- **The legacy loop checked the wrong sensors.** It ran over `0..N-1` while `sensor_t` runs over `1..N`, so it checked UNDEFINED_SENSOR and never checked the last sensor. On Twist the last sensor is ANALOG_COMM, whose threshold node is disabled, so nothing changes there. On a board whose last sensor has a limit, that limit is now checked.
- **The legacy reaction reconfigured the PWM pins through the GPIO API from the control interrupt.** That API takes a mutex, which is not allowed in an interrupt, and it overrode the leg safe states. The software check now latches a TRIP and calls `safety_emergency_stop()`, which works on registers only.
