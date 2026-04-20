!!! note ""
    Shield Position is the unified position-feedback API exposed as `shield.position`.
    It supports Hall effect sensors, incremental encoders, and Sin/Cos encoders connected through a shield.

## Validation Status

- Hall: validated on hardware 
- ABZ incremental encoder: validated on hardware
- Sin/Cos: API available, but still experimental and not yet hardware-validated

For upstreaming, Hall and ABZ are the validated sensor paths. Sin/Cos stays documented so the public API is complete, but it should not be presented as validation-backed yet.

## Include

```cpp
#include <ShieldAPI.h>
```

## Initialization Contract

The intended flow is the same for every sensor type:

1. Select the sensor with `shield.position.init(...)` or `shield.position.initDefault()`
2. Adjust runtime-owned parameters if needed
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

`shield.position.update(...)` returns `false` when the active sensor is not initialized correctly or when the current sample cannot be used.

## Common Outputs

Once initialized, every supported sensor type provides:

- `shield.position.getMechanicalAngle()`
- `shield.position.getElectricalAngle()`
- `shield.position.getMechanicalSpeed()`
- `shield.position.getElectricalSpeed()`
- `shield.position.getMechanicalPulsation()`
- `shield.position.getElectricalPulsation()`

## Configuration Ownership

Position configuration is split between:

- devicetree-owned defaults describing the shield wiring and default sensor behavior
- runtime-owned parameters that applications may override after `init()`

### Devicetree-owned defaults

These values are expected to live in shield or application overlays:

- sensor selection through `chosen`
- Hall GPIO wiring
- Sin/Cos sensor names and analog pin mapping
- ABZ timer selection
  - ABZ index presence
  - ABZ index polarity
  - ABZ index configuration
- per-sensor default direction sign
- per-sensor default electrical offset
- per-sensor default counts per revolution
- motor default pole pairs

### Runtime-owned overrides

The application can still tune:

- `setDirectionSign(...)`
- `setPolePairs(...)`
- `setElectricalOffset(...)`
- for ABZ
  - `setCountsPerRevolution(...)` for ABZ
  - `setAbzSpeedDecimation(...)` for ABZ
- for Hall
  - `setHallSectorTable(...)` for Hall
  - `setHallInterpolation(...)` for Hall

The ABZ index properties are intentionally read-only at runtime. They are treated as hardware configuration and should be declared in devicetree or application overlay.

--- 
## Hall Effect Sensor

Use Hall mode when the motor provides three digital Hall channels.

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

---
## Incremental Encoder

Use ABZ mode when the motor provides a quadrature encoder and the timer peripheral is wired in encoder mode.

### ABZ properties

The default ABZ sensor configuration includes:

- `timer`
- `counts-per-revolution`
- `direction-sign`
- `electrical-offset`
- `index-present`
- `index-polarity`
- `index-configuration`

Meaning of the index-specific fields:

- `index-present = <1>` enables the timer index input
- `index-present = <0>` keeps AB quadrature active without index reset
- `index-polarity = "NONINVERTED"` treats the index input as active high or rising edge
- `index-polarity = "INVERTED"` treats the index input as active low or falling edge
- `index-configuration` selects the AB phase state required for the active index pulse to reset the counter

Supported `index-configuration` values are:

- `"A_LOW_B_LOW"`
- `"A_LOW_B_HIGH"`
- `"A_HIGH_B_LOW"`
- `"A_HIGH_B_HIGH"`

---
### Devicetree example

```dts
/ {
    chosen {
        owntech,position-sensor = &abz;
    };
};

&default_motor {
    pole-pairs = <4>;
};

&abz {
    counts-per-revolution = <4096>;
    direction-sign = <1>;
    electrical-offset = <0x00000000>;
    index-present = <1>;
    index-polarity = "INVERTED";
    index-configuration = "A_HIGH_B_HIGH";
};
```
---
### Runtime example

```cpp
shield.position.init(ABZ);

shield.position.setCountsPerRevolution(4096);
shield.position.setDirectionSign(1);
shield.position.setPolePairs(4);
shield.position.setElectricalOffset(0.0F);
shield.position.setAbzSpeedDecimation(100);

shield.position.update(100e-6F);

float32_t mechanical_angle = shield.position.getMechanicalAngle();
float32_t electrical_angle = shield.position.getElectricalAngle();
float32_t mechanical_speed = shield.position.getMechanicalSpeed();
```

The raw encoder count remains available through:

```cpp
uint32_t encoder_count = shield.position.getIncrementalEncoderValue();
```

---
## Sin/Cos Encoder

Use Sin/Cos mode when the position sensor provides two analog channels: one sine and one cosine.

!!! warning
    `shield.position.init(SINCOS)` enables the semantic shield sensors, but converted Sin/Cos samples are only available once the ADC acquisition path has been started. Do not rely on `shield.position.update(...)` until the acquisition path is running.
    
    **Sin/Cos is still experimental in this upstreaming pass.**
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

---
## Default Sensor Selection

If the application always uses the same sensor, define it in `src/app.overlay`:

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
