# BLDC FOC With Selectable Position Sensor

This example runs the same cascaded control structure with any of the position sensors already wired on the Ownverter shield:

- Hall
- ABZ incremental encoder
- Sin/Cos analog sensor

The sensor selection is done entirely from the devicetree overlay. `src/main.cpp` now reads the active sensor through `PositionAPI` and only uses ABZ-specific count data when the selected sensor is actually `ABZ`.

## Files

- `src/hall.overlay`: turnkey Hall configuration
- `src/abz.overlay`: turnkey ABZ configuration
- `src/sincos.overlay`: turnkey Sin/Cos configuration
- `src/app.overlay`: active overlay used by the build
- `src/main.cpp`: state machine, task entry points, and runtime sequencing
- `src/app.h`: helper declarations plus the shared `AppContext`
- `src/app.cpp`: helper implementations used by `main.cpp`

`AppContext` is intentionally split into:

- `AppSetup`: values chosen for configuration and initialization
- `AppVariable`: values updated while the application is running

This keeps the state machine readable because setup-oriented variables are not
mixed with measurements, loop counters, and live control data.

## How To Select The Sensor

1. Copy one of the provided overlays onto `src/app.overlay`.
2. Adjust motor and sensor parameters if needed.
3. Build and flash.

Example:

```bash
cp src/abz.overlay src/app.overlay
```

## Overlay Content

Each overlay:

- selects the sensor through `/chosen/owntech,position-sensor`
- sets `&default_motor { pole-pairs = <...>; }`
- overrides the sensor-specific parameters on the shield node already declared by `ownverter_v1_1_0.overlay`

The hardware pin mapping remains the one defined by the shield:

- Hall: spin header pins `7`, `9`, `49`
- ABZ: timer `timers3`
- Sin/Cos: `ANALOG_SIN` on pin `43`, `ANALOG_COS` on pin `45`

## Parameters To Check

Before running the motor, verify these values in the chosen overlay:

- `pole-pairs`
- `direction-sign`
- `electrical-offset`
- `counts-per-revolution` for ABZ
- `hall-sector-table` and `hall-interpolation` for Hall

`electrical-offset` is stored as the raw IEEE754 `float32` bit pattern expected by the binding. `0x00000000` means `0.0f`.

## Runtime Behavior

`main.cpp` uses `shield.position.initDefault()` and expects the selected overlay to define the active sensor. The control only enters `POWER_ST` when:

- the position sensor initialized correctly
- the position update is valid
- the DC bus voltage is above the startup threshold

If position feedback becomes invalid while running, the control falls back to `ERROR_ST` instead of continuing with stale rotor angle data.

The runtime state machine and protections stay in `main.cpp`, while the FOC
math path is now delegated to the calculation-only `motor_control` library
under `owntech/lib/USB/motor_control`.

## Control Flow

```mermaid
flowchart TD
    A[main] --> B[setup_routine]
    B --> C[Init power stage in buck mode]
    C --> D[Enable default sensors]
    D --> E[Init selected position sensor from app.overlay]
    E --> F[Configure ScopeMimicry channels and trigger]
    F --> G[Reset filters, PIDs, variables, and start offset calibration]
    G --> H[Create background, application, and critical tasks]
    H --> I[Start scheduler tasks]

    I --> J[loop_background_task]
    I --> K[application_task]
    I --> L[loop_critical_task at 10 kHz]

    J --> J1[Read serial command]
    J1 --> J2{Command}
    J2 -->|p| J3[Request POWERMODE and restart scope]
    J2 -->|i| J4[Request IDLEMODE and zero speed_ref]
    J2 -->|o| J5[Restart offset calibration]
    J2 -->|u or d| J6[Adjust speed_ref within limits]
    J2 -->|r q m| J7[Control scope dump or replay]

    K --> K1{is_downloading}
    K1 -->|yes| K2[Dump recorded scope data]
    K1 -->|no| K3[Continue]
    K2 --> K3
    K3 --> K4{memory_print}
    K4 -->|false| K5[Print live status values]
    K4 -->|true| K6[Replay buffered scope samples]
    K5 --> K7{control_state}
    K6 --> K7
    K7 -->|OFFSET_ST| K8{counter_time > NB_OFFSET}
    K8 -->|yes| K9[Compute current offsets and go to IDLE_ST]
    K8 -->|no| K13[Stay in OFFSET_ST]
    K7 -->|IDLE_ST| K10{Power requested and position valid and V_high_filtered > V_HIGH_MIN}
    K10 -->|yes| K11[Enter POWER_ST]
    K10 -->|no| K14[Stay in IDLE_ST]
    K7 -->|POWER_ST| K12{asked_mode == IDLEMODE}
    K12 -->|yes| K14
    K12 -->|no| K15[Stay in POWER_ST]
    K7 -->|ERROR_ST| K16{asked_mode == IDLEMODE}
    K16 -->|yes| K17[Clear error counter and go to IDLE_ST]
    K16 -->|no| K18[Stay in ERROR_ST]

    L --> L1[Increment counter_time]
    L1 --> L2[retrieve_analog_datas]
    L2 --> L3[get_position_and_speed]
    L3 --> L4{POWER_ST and invalid position data}
    L4 -->|yes| L5[Force ERROR_ST]
    L4 -->|no| L6[Continue]
    L5 --> L6
    L6 --> L7[overcurrent_mngt]
    L7 --> L8{control_state}
    L8 -->|OFFSET_ST / IDLE_ST / ERROR_ST| L9[Stop PWM and reset filters/controllers if needed]
    L8 -->|POWER_ST| L10[Run speed PI every 10 ticks]
    L10 --> L11[Run d/q current control]
    L11 --> L12[Compute phase voltages]
    L12 --> L13[Convert voltages to duty cycles]
    L13 --> L14[Apply duties to LEG1 LEG2 LEG3]
    L14 --> L15[Start PWM if not already running]
    L9 --> L16{counter_time % decimation == 0}
    L15 --> L16
    L16 -->|yes| L17[Mirror control data and acquire scope sample]
    L16 -->|no| L18[Wait for next critical tick]
    L17 --> L18
```

## Serial Commands

- `p`: request power mode
- `i`: request idle mode
- `o`: restart current offset calibration
- `u`: increase speed reference
- `d`: decrease speed reference
- `r`: dump scope data
- `q`: restart scope acquisition
- `m`: toggle buffered scope replay

## Hall Notes

The default Hall sector table is:

```text
{5, 1, 0, 3, 4, 2}
```

This maps raw Hall states `001` to `110` to electrical sectors. If your motor phases or Hall wiring are permuted, this table or `direction-sign` may need adjustment.

## Phase Mapping

| Electrical phase | PWM  | LEG  |
|------------------|------|------|
| Phase A          | PWMA | LEG1 |
| Phase B          | PWMC | LEG2 |
| Phase C          | PWME | LEG3 |
