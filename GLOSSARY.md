# OwnTech Core

Firmware framework for the OwnTech Spin board and its power shields (Twist, Ownverter, uSolarVerter, O2 and future boards).

## Safety

**Protection path**:
The mechanism that enforces one limit: the comparator path, the ADC-watchdog path or the software path. The build picks the strongest one available for each limit.
_Avoid_: layer, safety level (L0/L1/L2 are test levels)

**Comparator path**:
A comparator watching a sensor pin and driving an HRTIM fault input, which turns the outputs off with no CPU involvement.
_Avoid_: hardware safety (unqualified), analog protection

**ADC-watchdog path**:
An ADC analog watchdog routed through an HRTIM external event to an HRTIM fault input. It needs no CPU, and it acts once per ADC conversion.
_Avoid_: AWD protection, software watchdog

**Software path**:
A limit checked by firmware in the control task, for what the hardware cannot watch: computed quantities, and sensors left without a free hardware channel.
_Avoid_: safety API (when meaning only this path), polling

**CPU supervision**:
The independent watchdog, the fatal-error handler and the HRTIM system fault. Together they bring the converter to its safe state when the CPU itself fails. Always on.
_Avoid_: watchdog (when meaning all three)

**Trip**:
A protection path or CPU supervision acting: every leg goes to its safe state, and the safe outputs go to their safe level. A trip latches until cleared.
_Avoid_: fault (when meaning the reaction), shutdown, alarm

**Fault**:
The condition that causes a trip: a limit crossed, a self-test failure, or a CPU failure.
_Avoid_: error, trip (when meaning the cause)

**Safe state**:
The state a leg's outputs take on a trip: open (all switches off, the default), short-low or short-high. It is set per leg, before the PWM starts.
_Avoid_: reaction, stop state

**Safe output**:
A GPIO declared in the shield devicetree, such as a relay or a driver enable, that the firmware drives to its safe level on a trip. The board's pull resistor must hold that level while the MCU resets.
_Avoid_: safety pin, emergency output

**Arming**:
Configuring every hardware trip, proving each one by a functional test, and locking the configuration until reset. `power.start()` arms automatically.
_Avoid_: enabling safety, initialising the safety API

**Limit**:
The value beyond which a quantity trips the converter. The shield devicetree sets its absolute maximum; user code may tighten it, never loosen it.
_Avoid_: threshold (when meaning the declared value), setpoint

**Protection manifest**:
The file the build writes from the shield devicetree and the fault-channel allocation. It lists every limit, its path, each leg's safe state and the safe outputs. It is the single source for the boot line, the docs table and the generated tests.
_Avoid_: safety config, protection map

**Fault reaction time**:
The declared time from a fault occurring to the outputs reaching the safe state, per protection path (IEC 60730-1 H.11).
_Avoid_: latency (when meaning the declared value), trip time

**Development mode**:
A build selected by `safety-mode = "development"` in `app.overlay`, in which limits may be loosened and the hardware paths disarmed. CPU supervision stays on.
_Avoid_: debug mode, unsafe mode

**Bench file**:
The one hand-written, board-specific input to the generated powered tests: where the fault switch goes, source limits, output ports and scope wiring.
_Avoid_: test config, board profile
