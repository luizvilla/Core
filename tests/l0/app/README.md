# L0-APP: application control step

Open-loop tests of `controlInit()` and `controlStep()` ([src/app/control_step.h](../../../src/app/control_step.h)), the hardware-free logic of the critical task. Catalogue: [procedure §6.4](../../../docs/testing/test-procedure.md#64-l0-app--application-control-step-no-model).

## Run

```bash
bash tests/l0/app/run.sh
tests/run_all.sh --l0
```

## What is compiled

| Source | Flags |
|---|---|
| `test_app.cpp` | strict (`-Werror`) |
| `src/app/control_step.cpp`, `src/boost_converter/boost.cpp`, `src/grid_observer/gridObserver.cpp` | strict |
| `single_phase_inverter/src/singlePhaseInverterFollowing.cpp`, `sogi.cpp` | library (no `-Werror`) |
| `control_library/src/pid.cpp`, `transform.cpp`, `trigo.cpp` | library |

These are exactly the sources the firmware compiles behind `controlStep()`.

## How the test drives the step

- **State:** every case builds its own `ControlState` with `controlInit()`, so it runs with the application's real tuning (`app::` constants), not test values.
- **Boost inputs:** fixed measurements, bus at 390 V (below the 400 V reference, inside the 395–405 V burst band) and 30 V on the PV side. Nothing responds to the duties: L0 has no plant.
- **Grid inputs:** a synthetic 230 V / 50 Hz sine, one sample per 100 µs period. A stale VAC is simulated the way the critical task sees it: `v_ac_updated = false` and the last value held.
- **Real time:** grid checks go through the real PLL → observer → inverter chain, so they use the real delays: about 1 s of healthy grid to qualify, 20 ms to declare VAC stale, 0.1 s to trip.
- **Operator commands** (`mode`, `sync_requested`, `v_bus_ref`) replace the serial menu.

## Criteria

| Test case | Criteria | Checks |
|---|---|---|
| L0-APP-01 | .a – .c | `controlInit` applies the app tuning; a mode change is flagged on its tick only and runs the cold loop on that tick; leg enables and duties are the controller's; IDLE leaves the legs untouched |
| L0-APP-02 | .a, .b | Cold loop on the change tick then every 10th tick (20 in 200 ticks); a new mode change restarts the phase |
| L0-APP-03 | .a – .c | VAC stale for exactly 200 periods trips the observer with `InvalidMeasurement`; one fresh sample restarts the count; one sample missing at boot does not latch a trip |
| L0-APP-04 | .a | Over every mode, with and without a sync request, `bridge_enable` stays false and the inverter never requests PWM or the relay |
| L0-APP-05 | .a, .d; INFO .b, .c | Synchronization only after the observer qualifies (and 1000 calls later); an under-voltage trip removes synchronization on the trip tick; times to qualification and synchronization |

## Known findings

| Criterion | Status | Issue |
|---|---|---|
| L0-APP-03.c | `FAIL`: one missing VAC sample on the first control step latches `InvalidMeasurement`; nothing calls `clearTrip()`, so the inverter never synchronizes until reset | [luizvilla/Core#2](https://github.com/luizvilla/Core/issues/2) |

## History

Added in the commit after `6ac9bc7`, which extracted `controlStep()` from the critical task. Planted bugs (off-by-one in the stale-VAC check, grid quality forced to true) turn L0-APP-03.a and L0-APP-05.a to `FAIL`.
