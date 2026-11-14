!!! Note ""
    Safety API is designed to provide safety features when working with the Spin board.

To work with Spin API, include the following file in your code:
!!! note
    ```
    #include <SafetyAPI.h>
    ```


## How the protection works

The shield's devicetree declares the **limits** of each sensor: the board's absolute maximum. At build time, each limit gets the strongest path the board offers:

- a **comparator**, which trips in less than 0.5 µs;
- an **ADC watchdog**, which trips within 3 switching periods;
- the **software check**, which trips within 5 control periods.

The comparator and ADC-watchdog paths act on the PWM in hardware, even if the CPU stops. The build prints the path given to each limit. Each board's table is in `docs/safety/boards/`.

Your code does not need any safety call. To make a limit stricter, use:

```cpp
safety.setTripThreshold(I1_LOW, -5.0, 5.0);   // can only tighten a limit
```

## When a trip happens

On any trip, hardware or software:

- every leg takes its **safe state**: open by default, or `safe-state` in the devicetree;
- safe outputs such as relays take their safe level;
- the fault stays **latched**, even across a reset.

The PWM stays refused until you call `safety.clearFault()` with the PWM off. `safety.getFaultDescription()` tells you what happened and how to clear it, for example:

```
software trip: V_HIGH 52.3 above high limit; clear: PWM off, clearFault()
```

The older functions still work:

- `setChannelReaction()` sets the safe state of every leg. Call it before the PWM is initialised.
- `setChannelThresholdMax()` / `Min()` and the thresholds stored in flash can only be **tighter** than the board's limits. A looser value is refused.

## Development mode

For bench bring-up only, add this to `app.overlay`:

```
/ {
    safety-config {
        compatible = "safety-config";
        safety-mode = "development";
    };
};
```

Only in this mode:

- `safety.disarm()` compiles: it leaves the hardware trips unarmed;
- software limits may be loosened;
- `disableSafetyApi()` stops the software check.

The firmware prints a banner at boot. CPU supervision stays on.

!!! warning
    Never flash a development build on a deployed converter. In any other build, `safety.disarm()` does not compile.

## API Reference
::: doxy.powerAPI.class
name: SafetyAPI
