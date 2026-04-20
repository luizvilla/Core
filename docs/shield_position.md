!!! note ""
    Shield Position is the unified position-feedback API exposed as `shield.position`.
    It currently supports three sensor families connected through a shield:
    Hall effect sensors, incremental encoders (ABZ), and Sin/Cos encoders.

## Upstream Status

- Hall: validated on hardware
- ABZ incremental encoder: validated on hardware
- Sin/Cos: API implemented, but not yet validated on hardware

For upstreaming, Hall and ABZ should be treated as validated paths. Sin/Cos should remain documented as experimental until the validation work is complete.

## Include

```cpp
#include <ShieldAPI.h>
```

## Initialization Flow

The recommended sequence is the same for all sensor types:

1. Select the sensor with `shield.position.init(...)` or `shield.position.initDefault()`
2. Override runtime-tunable parameters if needed
3. Call `shield.position.update(sample_time)` periodically
4. Read angle and speed outputs

```cpp
shield.position.init(ABZ);
shield.position.setCountsPerRevolution(4096);
shield.position.setPolePairs(4);

shield.position.update(100e-6F);

float32_t electrical_angle = shield.position.getElectricalAngle();
float32_t electrical_speed = shield.position.getElectricalSpeed();
```

## Common Outputs

Once initialized, all supported sensor types provide the same estimated outputs:

- `shield.position.getMechanicalAngle()`
- `shield.position.getElectricalAngle()`
- `shield.position.getMechanicalSpeed()`
- `shield.position.getElectricalSpeed()`
- `shield.position.getMechanicalPulsation()`
- `shield.position.getElectricalPulsation()`

## Configuration Ownership

The current API splits configuration between:

- devicetree defaults stored by the shield description
- runtime setters applied by the application after `init()`

Today, the runtime setters are:

- `setDirectionSign(...)`
- `setPolePairs(...)`
- `setElectricalOffset(...)`
- `setCountsPerRevolution(...)` for ABZ
- `setAbzSpeedDecimation(...)` for ABZ
- `setHallSectorTable(...)` for Hall
- `setHallInterpolation(...)` for Hall

The corresponding getters allow the application to inspect the active values after initialization.

## Hall Effect Sensor

Use the Hall mode when the motor exposes three digital Hall signals.

Typical parameters:

- `direction-sign`
- `pole-pairs`
- `electrical-offset`
- `hall-sector-table`
- `hall-interpolation`

Minimal sequence:

```cpp
shield.position.init(HALL);

uint8_t hall_table[6] = {5, 1, 0, 3, 4, 2};
shield.position.setPolePairs(4);
shield.position.setElectricalOffset(0.0F);
shield.position.setHallSectorTable(hall_table);
shield.position.setHallInterpolation(HALL_INTERPOLATION_LINEAR);

shield.position.update(100e-6F);
```

The raw Hall state remains available through:

```cpp
uint8_t hall_state = shield.position.getHallState();
```

## Incremental Encoder

Use the ABZ mode when the motor exposes an incremental encoder and the timer peripheral is wired in encoder mode.

Typical parameters:

- `counts-per-revolution`
- `direction-sign`
- `pole-pairs`
- `electrical-offset`

Minimal sequence:

```cpp
shield.position.init(ABZ);
shield.position.setCountsPerRevolution(4096);
shield.position.setDirectionSign(1);
shield.position.setPolePairs(4);
shield.position.setElectricalOffset(0.0F);

shield.position.update(100e-6F);
```

The raw encoder count remains available through:

```cpp
uint32_t encoder_count = shield.position.getIncrementalEncoderValue();
```

### Current upstream gap

The current ABZ implementation already relies on timer index behavior in the STM32 timer driver, but those index settings are not yet exposed through the Position API configuration surface.

The missing ABZ settings that should be surfaced in follow-up commits are:

- index presence
- index polarity
- index reset configuration

## Sin/Cos Encoder

Use the Sin/Cos mode when the position sensor provides two analog channels: one sine and one cosine.

!!! warning
    `shield.position.init(SINCOS)` enables the semantic shield sensors, but converted Sin/Cos samples are only available once the ADC acquisition path has been started. Do not rely on `shield.position.update(...)` until the data acquisition path is running.

Sin/Cos is part of the API surface, but it is not yet validation-backed for upstreaming.

Typical parameters:

- `direction-sign`
- `pole-pairs`
- `electrical-offset`

Minimal sequence:

```cpp
shield.position.init(SINCOS);
shield.position.setDirectionSign(1);
shield.position.setPolePairs(4);
shield.position.setElectricalOffset(0.0F);

shield.position.update(100e-6F);
```

Raw analog channels remain available through:

```cpp
float32_t sin_value = shield.position.getSinValue();
float32_t cos_value = shield.position.getCosValue();
```

## Default Sensor Selection

If the application always uses the same position sensor, it can be selected in `src/app.overlay`:

```dts
/ {
    chosen {
        owntech,position-sensor = &abz;
    };
};
```

Then initialize it with:

```cpp
shield.position.initDefault();
```

## Current Devicetree Defaults

Applications can override the shield defaults from `src/app.overlay`.

Hall example:

```dts
&default_motor {
    pole-pairs = <4>;
};

&hall {
    direction-sign = <1>;
    electrical-offset = <0x00000000>;
    hall-sector-table = <5 1 0 3 4 2>;
    hall-interpolation = "LINEAR";
};
```

ABZ example:

```dts
&default_motor {
    pole-pairs = <4>;
};

&abz {
    counts-per-revolution = <4096>;
    direction-sign = <1>;
    electrical-offset = <0x00000000>;
};
```

Sin/Cos example:

```dts
&default_motor {
    pole-pairs = <4>;
};

&sincos {
    direction-sign = <1>;
    electrical-offset = <0x3dcccccd>;
};
```
