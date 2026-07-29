# ThingSet Serial Power Test Bench Example

This example exposes a Twist converter over the
[ThingSet](https://thingset.io) Text Mode protocol through a dedicated
USB-CDC Zephyr shell. It replaces the positional ASCII commands used by the
older `comm_protocol` / `python_comm_library` example with a structured object
tree and ThingSet write callbacks.

The application can remotely configure both power legs, select manual-duty or
PID control, choose a PID tracking measurement, update timing and switch
settings, calibrate sensor channels, and read live measurements and control
readbacks.

## Runtime architecture

The application separates control-loop work from immediate configuration side
effects:

- The background task blinks the LED, samples both temperature sensors, and
  uses `/Config/wBlinkPeriod_s` as its delay.
- The 100 us critical task samples voltage/current channels, starts or stops
  each PWM leg according to `Mode` and `wEnable`, calculates manual or PID duty,
  applies the duty, and updates peak-voltage and duty readbacks.
- ThingSet group callbacks immediately apply capacitor, driver, phase-shift,
  dead-time, switching-frequency, and calibration writes.

ThingSet registration lives in one translation unit so every object is
registered exactly once.

## Files

- `main.cpp` — setup, background sampling, critical control loop, and two PID
  instances.
- `user_data_objects.h` — shared types, object IDs, callback declarations, and
  `extern` storage declarations.
- `user_data_objects.cpp` — backing storage and all `THINGSET_ADD_*`
  registrations.
- `thingset_callbacks.cpp` — write validation and immediate Shield/Spin API
  side effects.
- `app.conf` — enables ThingSet and its shell transport.
- `app.overlay` — provides a second USB-CDC UART dedicated to the ThingSet
  shell.
- `tools/thingset_tools.py` and `tools/ThingSetTools.m` — generic Python and
  MATLAB ThingSet transports.
- `tools/power_test_bench.py` and `tools/PowerTestBench.m` — matched,
  safety-oriented converter clients.
- `tools/thingset_example.py` and `tools/thingset_example.m` — safe-by-default
  command-line and callable examples.
- `tools/tests` — Python `unittest` and MATLAB `matlab.unittest` suites using
  fake transports, with no hardware side effects.

`../zephyr/CMakeLists.txt` explicitly compiles `main.cpp`,
`user_data_objects.cpp`, and `thingset_callbacks.cpp`.

## ThingSet object tree

### Converter metadata

`/Converter` is application-owned writable identity metadata:

| Path | Maximum length | Compiled default |
| --- | ---: | --- |
| `/Converter/wBoardName` | 23 characters | Shield devicetree name (`TWIST`) |
| `/Converter/wBoardVersion` | 15 characters | Shield devicetree version (`v1.4.2`) |
| `/Converter/wSerialNumber` | 47 characters | `UNSET` |
| `/Converter/wFirmwareVersion` | 31 characters | `1.0.0` |

Values must be non-empty printable ASCII. A successful write persists the
entire metadata record under application NVS key `0x0401`; it is restored
during startup. Invalid data, an incompatible stored record, or an NVS write
failure leaves the previous complete record in place.

The OwnTech communication module separately supplies `/Device` with read-only
framework metadata. `/Converter` avoids duplicate ThingSet IDs while providing
the mutable, application-owned converter identity required by this example.

### Measurements

All measurement objects are read-only.

| Path | Description |
| --- | --- |
| `/Measurements/rV1Low_V`, `/Measurements/rV2Low_V` | Low-side leg voltages |
| `/Measurements/rVHigh_V` | High-side or bus voltage |
| `/Measurements/rI1Low_A`, `/Measurements/rI2Low_A` | Low-side leg currents |
| `/Measurements/rIHigh_A` | High-side or bus current |
| `/Measurements/rTemp1_degC`, `/Measurements/rTemp2_degC` | Twist temperatures |
| `/Measurements/rV1Max_V`, `/Measurements/rV2Max_V` | Peak leg voltage while that leg is running |
| `/Measurements/rDuty1`, `/Measurements/rDuty2` | Last duty value applied to each leg |

The peak readback resets to zero when its leg stops.

### Global configuration

| Path | Type | Description |
| --- | --- | --- |
| `/Config/wBlinkPeriod_s` | float | LED blink half-period in seconds |
| `/Config/Mode` | uint8 | Global tester state |
| `/Config/Frequency_Hz` | uint32 | Global PWM frequency; applied immediately after range validation |

Mode values preserve the original power-test-bench protocol:

| Value | Mode | Power behavior |
| --- | --- | --- |
| `0` | `IDLE` | Both legs stopped |
| `1` | `POWER_ON` | An enabled leg may run |
| `2` | `POWER_OFF` | Both legs stopped while configuration and measurements remain available |

At startup the application is in `IDLE`, uses a 200 kHz switching frequency,
and leaves both legs disabled.

### Per-leg configuration

`/Config/Leg1` and `/Config/Leg2` expose the same fields:

| Item | Type | Behavior |
| --- | --- | --- |
| `wEnable` | bool | Allows the leg to run when `Mode` is `POWER_ON` |
| `wCapa` | bool | Immediately connects or disconnects the leg capacitor |
| `wDriver` | bool | Immediately connects or disconnects the gate driver |
| `wBuck` | bool | Enables PID-derived duty using the normal duty convention |
| `wBoost` | bool | Enables PID-derived duty and applies `1 - duty` |
| `wDutyCycle` | float | Manual duty in the range 0–1 |
| `wReferenceValue` | float | PID setpoint |
| `wTrackingVar` | string | PID measurement: `V1`, `V2`, `VH`, `I1`, `I2`, or `IH` |
| `wPhaseShift` | int16 | Phase shift in degrees, from -360 to 360 |
| `wDeadTimeRising_ns` | uint16 | Rising-edge dead time in nanoseconds |
| `wDeadTimeFalling_ns` | uint16 | Falling-edge dead time in nanoseconds |

When both `wBuck` and `wBoost` are false, the critical task applies
`wDutyCycle` directly. Setting either flag enables PID calculation; `wBoost`
also inverts the resulting duty. Client applications should treat `wBuck` and
`wBoost` as mutually exclusive.

Invalid duty, phase-shift, tracking-variable, mode, or frequency writes are
restored to their preceding value.

The two PID instances use the original example's fixed defaults: a 100 us
sample time, `Kp = 0.000215`, `Ti = 7.5175e-5`, zero derivative/filter terms,
and output bounds of 0–1.

### Calibration

Each channel under `/Calibration/V1`, `/Calibration/V2`, `/Calibration/VH`,
`/Calibration/I1`, `/Calibration/I2`, and `/Calibration/IH` contains:

| Item | Type | Behavior |
| --- | --- | --- |
| `wGain` | float | Applies the channel's linear-conversion gain |
| `wOffset` | float | Applies the channel's linear-conversion offset |
| `wStore` | bool | On `true`, stores parameters in non-volatile memory, reads them back for verification, then resets to `false` |

Gain and offset writes take effect immediately. They are not persisted until
`wStore` is triggered.

## Immediate and deferred writes

| Timing | Objects |
| --- | --- |
| Immediate callback | Converter metadata, `Frequency_Hz`, `wCapa`, `wDriver`, `wPhaseShift`, both dead times, calibration gain/offset/store |
| Critical-task evaluation | `Mode`, `wEnable`, `wBuck`, `wBoost`, `wDutyCycle`, `wReferenceValue`, `wTrackingVar` |

`Mode` and `wEnable` are both required to start PWM. Moving out of `POWER_ON`
or clearing `wEnable` stops the corresponding leg on the next critical-task
tick.

## Configuration and USB transport

`app.conf` enables:

```text
CONFIG_OWNTECH_COMMUNICATION_ENABLE_CAN=y
CONFIG_THINGSET_SHELL=y
```

`CONFIG_OWNTECH_COMMUNICATION_ENABLE_CAN` is currently the repository switch
that selects the ThingSet stack, even though this example uses the serial shell
rather than CAN.

`app.overlay` adds a second CDC-ACM UART and routes `zephyr,shell-uart` to it.
After flashing, the board therefore enumerates separate console and ThingSet
shell serial ports.

## Build and flash

From the repository root:

```sh
~/.platformio/penv/bin/pio run -e USB
~/.platformio/penv/bin/pio run -e USB -t upload
```

The `control_library` dependency in `platformio.ini` supplies the `Pid` and
`PidParams` implementation.

## Manual shell use

> **Power hardware warning:** Writes to capacitor, driver, phase shift, dead
> time, frequency, and calibration objects act immediately, even when the
> global mode is not `POWER_ON`. Use an isolated, current-limited bench setup
> and the normal Twist power-stage safety procedure.

Open the ThingSet shell port, for example `/dev/ttyACM1`, at 115200 baud and
select the ThingSet command context:

```text
select thingset
```

Discovery and reads:

```text
?
?Measurements null
?Converter null
?Config null
?Config/Leg1 null
?Calibration null
?Measurements/rV1Low_V
?Measurements/rDuty1
```

Read or update persistent converter identity:

```text
?Converter
=Converter {\"wSerialNumber\":\"TWIST-001\",\"wFirmwareVersion\":\"1.0.1\"}
```

Keep the converter off while configuring it:

```text
=Config {\"Mode\":2}
=Config/Leg1 {\"wDutyCycle\":0.2,\"wEnable\":true}
=Config/Leg1 {\"wTrackingVar\":\"V1\",\"wReferenceValue\":5.0,\"wBuck\":true}
```

Only on a prepared power bench, immediate switch writes and power-on can then
be issued:

```text
=Config/Leg1 {\"wCapa\":true,\"wDriver\":true}
=Config {\"Mode\":1}
?Measurements/rV1Max_V
=Config {\"Mode\":2}
```

Calibration example:

```text
=Calibration/V1 {\"wGain\":1.0,\"wOffset\":0.0}
=Calibration/V1 {\"wStore\":true}
```

The Zephyr shell strips unescaped double quotes before ThingSet receives the
request. A command such as:

```text
=Config {"Mode":2}
```

therefore returns `:A0` (Bad Request). Escape quotes as shown in the examples
above. Successful updates return `:84` (Changed).

## Python client

`tools/thingset_tools.py` requires `pyserial` and handles port detection,
`select thingset`, discovery, shell quote escaping, and generic reads/writes.
Discovery has exact access overrides for `Config/Mode` and
`Config/Frequency_Hz`, so `write_values()` and the attribute proxy correctly
treat both non-prefixed objects as writable.

`tools/power_test_bench.py` composes that transport into a validated
power-bench API:

```python
from power_test_bench import PowerTestBench
from thingset_tools import ThingSetTools

with ThingSetTools() as ts:
    ts.discover()
    bench = PowerTestBench(ts)

    bench.shutdown()
    bench.set_frequency(100000)
    bench.configure_leg(1, duty_cycle=0.2, phase_shift=0)

    print(bench.read_metadata())
    print(bench.read_measurements())

    # Only on a prepared, current-limited power bench:
    try:
        bench.power_on(1, connect_driver=True, duty_cycle=0.2)
    finally:
        bench.shutdown(disconnect_hardware=True)
```

`ThingSetTools()` probes candidate ports, so it can distinguish the shell port
from the console even though both interfaces share a USB VID/PID.
`discover()` writes `thingset_objects.json` and populates `ts.objects` for
attribute-style access and tab completion. `read_all()` and `write_values()`
operate on the discovered tree and reject read-only write targets locally.

`PowerTestBench` provides `set_mode`, `set_frequency`, `configure_leg`,
`read_leg`, `read_measurements`, `read_calibration`, `set_calibration`,
`read_metadata`, `set_metadata`, `power_on`, and `shutdown`. It validates
ranges and names, writes related fields as one group update, then reads them
back to detect firmware rejection. Setting converter metadata writes NVS on
the device. Calibration `store=True` verifies that `wStore` resets, but the
firmware exposes no separate storage-status object.

`power_on()` first requests `POWER_OFF`, disables both legs, configures and
enables exactly the selected leg, then requests `POWER_ON`. On failure it
attempts a shutdown. `shutdown()` always attempts `POWER_OFF` and both leg
disables; it leaves capacitor/driver switches unchanged unless
`disconnect_hardware=True`.

Run the bundled example from `src/tools`:

```sh
# Safe default: discovers, reads, configures a disabled leg, and remains off.
python3 thingset_example.py

# Explicit opt-in on a properly prepared power bench.
python3 thingset_example.py --leg 1 --duty 0.2 --duration 1 \
    --enable-power --connect-driver --connect-capacitor
```

## MATLAB client

`tools/ThingSetTools.m` provides the equivalent `discover`, `read`, `write`,
`readAll`, `writeValues`, and `fetchChildren` transport methods.
`tools/PowerTestBench.m` provides matching safety, validation, readback, and
metadata behavior:

```matlab
ts = ThingSetTools();
ts.discover();
bench = PowerTestBench(ts);

bench.shutdown();
bench.setFrequency(100000);
bench.configureLeg(1, struct("dutyCycle", 0.2, "phaseShift", 0));

disp(bench.readMetadata());
disp(bench.readMeasurements());
```

The MATLAB wrapper methods are `setMode`, `setFrequency`, `configureLeg`,
`readLeg`, `readMeasurements`, `readCalibration`, `setCalibration`,
`readMetadata`, `setMetadata`, `powerOn`, and `shutdown`. Configuration,
calibration, and metadata setters accept scalar structs with camelCase field
names.

Call the bundled example from `src/tools`:

```matlab
% Safe default.
thingset_example()

% Explicit power opt-in.
thingset_example( ...
    Leg=1, DutyCycle=0.2, Duration=1, ...
    EnablePower=true, ConnectDriver=true, ConnectCapacitor=true)
```

Unlike the Python helper, the MATLAB client does not expose the
attribute-proxy tree because ThingSet names do not always map cleanly to valid
MATLAB struct field names.

## Host-side tests

The unit tests use fake transports and never contact or energize hardware:

```sh
python3 -m unittest discover -s src/tools/tests -p 'test_*.py' -v

matlab -batch "addpath('src/tools','src/tools/tests'); \
results=runtests('src/tools/tests'); assertSuccess(results);"
```

The firmware build and fake-client tests do not replace hardware acceptance.
After flashing, separately verify metadata persistence across a reboot,
frequency range handling, measurement readback, explicit power-on sequencing,
and shutdown on interruption using the normal isolated bench procedure.
