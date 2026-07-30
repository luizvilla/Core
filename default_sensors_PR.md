# Default Sensor API Example Compatibility Plan

Status: plan only; no example implementation changes have been made.

## Decision

The compatibility fix is a direct call-site replacement.

The current loaded example fails at `src/main.cpp:118` with:

```cpp
shield.sensors.enableDefaultTwistSensors();
```

It compiles when that line is replaced with:

```cpp
shield.sensors.enableDefaultSensors();
```

The new API is shield-independent and obtains each sensor's ADC and acquisition
order from the shield devicetree. The call must remain in the same setup
position, after the power mode is initialized and before tasks or ADC
acquisition are started. Do not keep both calls or add a second initialization
call between lines 118 and 119.

The same replacement applies to the removed Ownverter-specific method:

```cpp
shield.sensors.enableDefaultOwnverterSensors();
```

No changes to sensor names, readout calls, temperature-trigger calls, task
ordering, or `app.ini` shield selection are required for this API migration.

## Scope and inventory

The Core change under test is commit `ccca4804`, which removed
`enableDefaultTwistSensors()` and `enableDefaultOwnverterSensors()` and exposed
`enableDefaultSensors()` instead.

The sibling `../examples` repository contains 29 tracked Twist and Ownverter
`main.cpp` files:

- 27 files contain obsolete default-sensor calls and need modification.
- There are 28 call sites: 22 Twist calls and 6 Ownverter calls.
- `TWIST/Communication/python_comm_library/main.cpp` contains one call for each
  shield behind preprocessor branches; both must be replaced.
- Two examples contain no default-sensor call and require no change:
  `OWNVERTER/basic/power_open_loop_manual_duty/main.cpp` and
  `OWNVERTER/basic/sine_modulation_ot_sin/main.cpp`.
- No Twist or Ownverter example README or support file contains either obsolete
  method name.

### Files to update

Ownverter:

- `OWNVERTER/BLDC_hall_sensor/main.cpp`
- `OWNVERTER/FOC_hall_sensor/main.cpp`
- `OWNVERTER/basic/measurements_enable/main.cpp`
- `OWNVERTER/three phase inverter/grid following/main.cpp`
- `OWNVERTER/three phase inverter/grid forming/main.cpp`

Twist:

- `TWIST/Basic/Open-Loop PWM/main.cpp`
- `TWIST/Basic/measurements_enable/main.cpp`
- `TWIST/Communication/CAN/main.cpp`
- `TWIST/Communication/RS485/main.cpp`
- `TWIST/Communication/ThingSet_Serial/main.cpp`
- `TWIST/Communication/python_comm_library/main.cpp`
- `TWIST/DC_DC/boost_voltage_mode/main.cpp`
- `TWIST/DC_DC/buck_current_mode/main.cpp`
- `TWIST/DC_DC/buck_voltage_mode/main.cpp`
- `TWIST/DC_DC/current_ripple_measurement/main.cpp`
- `TWIST/DC_DC/independent/main.cpp`
- `TWIST/DC_DC/interleaved/main.cpp`
- `TWIST/DC_DC/scope_simple_example/main.cpp`
- `TWIST/Inverter/DQ/Grid Following/main.cpp`
- `TWIST/Inverter/DQ/Grid Forming/main.cpp`
- `TWIST/Inverter/DQ/Open-Loop/main.cpp`
- `TWIST/Inverter/Proportional Resonant/grid_following/main.cpp`
- `TWIST/Inverter/Proportional Resonant/grid_forming/main.cpp`
- `TWIST/Microgrid/AC_client_server/main.cpp`
- `TWIST/Microgrid/AC_peer_to_peer/main.cpp`
- `TWIST/Microgrid/DC_client_server/main.cpp`
- `TWIST/Microgrid/DC_droop/main.cpp`

`Core/src/main.cpp` is a loaded working copy of the buck-voltage example and is
not the source of truth for the examples repository. Use it for representative
validation, but do not include its existing user changes in the examples
migration commit.

## Investigation and build evidence

All 29 examples were copied one at a time into an isolated detached worktree at
Core commit `ccca4804` and built in the USB environment with PlatformIO 6.1.19.
Neither repository's example sources were edited for these tests.

### Unmodified examples

- 2 of 29 passed. These are the two Ownverter examples that do not call a
  default-sensor initializer.
- 27 of 29 failed.
- 24 failed directly because `SensorsAPI` no longer has the shield-specific
  method; the compiler suggested `enableDefaultSensors()`.
- 3 reached an unrelated pre-existing build error before reaching the obsolete
  sensor call.

The currently loaded `TWIST/DC_DC/buck_voltage_mode` example was also built in
place. Its unmodified line 118 produced the expected missing-method error.

### Temporary rename-only validation

A temporary mechanical replacement of both obsolete identifiers was applied
only inside the detached build worktree:

- 25 of 29 examples were verified to build successfully. This includes
  `TWIST/DC_DC/buck_voltage_mode` and
  `TWIST/Communication/python_comm_library`; the latter passed on an isolated
  rerun after a transient sequential dependency-install state.
- 4 of 29 still stop at known errors unrelated to the Sensor API.
- No example produced a Sensor API compile error after the temporary
  replacement.

The four unrelated blockers are:

| Example | Existing blocker |
| --- | --- |
| `OWNVERTER/three phase inverter/grid following` | `ThreePhaseInverter.h` is not provided by its resolved dependencies. |
| `OWNVERTER/three phase inverter/grid forming` | `ThreePhaseInverter.h` is not provided by its resolved dependencies. |
| `TWIST/Inverter/DQ/Grid Following` | `singlePhaseInverter::init(...)` supplies five arguments, while the resolved library requires six, including `modulation_scheme`. |
| `TWIST/Inverter/DQ/Grid Forming` | `singlePhaseInverter::init(...)` supplies five arguments, while the resolved library requires six, including `modulation_scheme`. |

These failures predate and are independent of the default-sensor rename. They
should be reported with the validation result but not folded into this narrowly
scoped compatibility change.

Core documentation also contains stale references to
`enableDefaultTwistSensors()` in `docs/shield_sensors.md` and
`docs/shield_introduction.md`. Those pages are outside the requested examples
scope and should be handled in a separate documentation change unless the scope
is explicitly expanded.

## Implementation plan

1. Record this plan by itself as Commit 0 before changing any example source.
   Stage only `default_sensors_PR.md`; preserve all existing modified and
   untracked files in both repositories.
2. In the 27 listed files in `../examples`, replace every
   `enableDefaultTwistSensors()` and `enableDefaultOwnverterSensors()` call with
   `enableDefaultSensors()`. Preserve whitespace, surrounding conditionals,
   setup order, comments, and all other behavior.
3. Search the complete Twist and Ownverter trees to prove that neither obsolete
   identifier remains and that all 28 affected call sites now use
   `enableDefaultSensors()`.
4. Rebuild every one of the 29 tracked Twist and Ownverter examples against the
   cherry-picked Core commit, using each example's own `app.ini`, `app.conf`,
   and library dependencies. Keep build artifacts and logs outside both Git
   diffs.
5. Confirm that the 25 currently buildable examples pass. For the four known
   blockers, confirm that the failure remains the same unrelated dependency or
   control-library error and that no default-sensor diagnostic appears.
6. Review both working trees before committing. The examples diff must contain
   only the 28 method-name replacements in the 27 planned files, and the Core
   diff for Commit 0 must contain only this plan.

## Acceptance criteria

- The current buck-voltage call at line 118 is replaced, not supplemented, and
  its example compiles.
- All 28 obsolete call sites in `../examples` use
  `shield.sensors.enableDefaultSensors()`.
- No `enableDefaultTwistSensors` or `enableDefaultOwnverterSensors` reference
  remains anywhere under `../examples/TWIST` or `../examples/OWNVERTER`.
- The two examples without a default-sensor initializer remain unchanged.
- The 25 examples without unrelated blockers compile successfully.
- The four known unrelated failures are documented and show no Sensor API
  regression.
- Existing user modifications and the untracked
  `TWIST/Inverter/Grid Following/` directory remain untouched.

## Ordered atomic commit sequence

0. **Core repository — `docs: plan default sensor example migration`**
   Add only `default_sensors_PR.md`. This commit must precede implementation.
1. **Examples repository — `fix: use generic default sensor initializer`**
   Apply only the 28 call-site replacements across the 27 listed `main.cpp`
   files. Keep this as one atomic compatibility commit because the old methods
   were removed together and the examples otherwise cannot compile against the
   new Core API.

