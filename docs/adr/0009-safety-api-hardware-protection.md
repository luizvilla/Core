---
status: accepted
date: 2026-11-14
---

# Safety API: hardware protection paths, CPU supervision and DTS-generated tests

Decided by Luiz Villa on 2026-11-14, with Claude, after the bench incident of 2026-11-13 (branch `apf_quadrature`, firmware `019d33c`). The decision covers every OwnTech power shield, not one board. It is numbered 0009 to follow the ADRs of `apf_quadrature`, which this branch will be merged into.

## Context

**The incident.** During L2-INV-13 on the uSolarVerter (400 V bus, grid through a series bulb), both transistors of one bridge leg were destroyed. The console fell silent when the firmware scope dump started. On the bare Spin, the same freeze reproduces as `K_ERR_STACK_CHK_FAIL` in the background task (a 1 KiB stack overflowed by `sprintf`). Zephyr parks the CPU in `arch_system_halt()`, and the HRTIM goes on switching at its last duty. Every protection was software: the bench current trip, the grid observer, and the Safety API, which polls `peekLatestValue()` from the control task and calls `stop()` after 5 bad readings. All of them stopped together with the CPU.

**The current Safety API** (`zephyr/modules/owntech_safety_api`) has these properties:
- it is software only;
- user code can switch it off (`disableSafetyApi()`);
- user code can loosen any threshold, including thresholds stored in NVS;
- the reaction (open or short circuit) is chosen globally.

The HRTIM driver configures no fault input, and nothing uses a watchdog.

**What the STM32G474 offers** (RM0440 Rev 8, DS12288 Rev 6):

| Feature | Fact | Source |
|---|---|---|
| HRTIM fault inputs FLT1–6 | Disable the outputs with no CPU involvement; the source is a pin, a comparator or an external event | RM0440 §27.3.15, Table 82 |
| Fault source map | FLT1: PA12 / COMP2 / EEV1 · FLT2: PA15 / COMP4 / EEV2 · FLT3: PB10 / COMP6 / EEV3 · FLT4: PB11 / COMP1 / EEV4 · FLT5: PB0, PC7 / COMP3 / EEV5 · FLT6: PC10 / COMP5 / EEV6 | RM0440 Table 82 |
| ADC watchdog → external event | adc1_AWD1–3 → EEV1–3, adc2_AWD1–3 → EEV4–6 (EExSRC = 3), so an ADC watchdog can drive FLT1–6 | RM0440 Table 220 |
| Latency, comparator pin → HRTIM pin | 16 ns typ, 31 ns max (digital fault pin: 9 / 20 ns) | DS12288 Table 60 |
| Locks | FLTxLCK freezes a fault input's set-up and FLTLCK freezes which timers each fault stops, both until reset; COMPx LOCK makes COMPx_CSR read-only until reset; FAULTx[1:0] cannot change while the timer runs | RM0440 §27.3.15, §24.3.4, OUTxR |
| Fault output state | Per output: Inactive, Active or High-Z | RM0440 OUTxR FAULT1/2 |
| System fault | Cortex lockup, PVD, clock security, SRAM parity and flash ECC disable every fault-protected output | RM0440 §27.3.15 |
| Comparator | One direction per comparator (POL only inverts it); the inverting input is ¼, ½, ¾ or 1 × VREFINT, a DAC, or a pin; blanking comes only from general-purpose timers | RM0440 Tables 196–198 |
| ADC overrun | With OVRMOD = 0, unread data makes new conversions get discarded; OVRMOD = 1 overwrites | RM0440 §21.4 |

**Which sensors reach a comparator** (devicetree of every shield, Spin header map):

| Signal | MCU pin | Comparator | Boards |
|---|---|---|---|
| I1_LOW / I_LOW_1 | PA1 | COMP1+ | all |
| I2_LOW / I_LOW_2 | PC1 | COMP3+ (shared with PA0) | all |
| Ownverter third leg current | PB11 | COMP6+ | Ownverter |
| V1_LOW / V2_LOW / V_LOW | PA0 | COMP3+ (shared with PC1) | all |
| I_HIGH / I_AC | PC2 or PC3 | **none** | all |
| V_HIGH / V_DC_BUS | PC2 or PC3 | **none** | all |

On today's boards, peak-current mode already uses COMP1 (DAC3_CH1) and COMP3 (DAC1_CH1).

**The standards** (`/data/partage/OwnTech/2-Technical/12-Standards`):
- **IEC 62109-1 §15 and Annex B.** Firmware performing a safety function (over-current, synchronization of AC sources, connection/disconnection, emergency stop, temperature: B.2.1.4) is evaluated in one of two ways. Either (a) every abnormal condition is tested with all software limits disabled, and the hardware must protect alone, or (b) it complies with IEC 60730-1 Annex H.
- **IEC 60730-1 Annex H:**
  - Class B "prevents an unsafe state; its failure does not lead directly to a hazard" (H.3.21.2).
  - Its structure is a single channel with a functional test, or with periodic self-test (H.9.12.1.2.2).
  - Detection and reaction times are declared (H.8, H.11), and safety data is protected from the user (H.9.12.2.10).
  - Behaviour on a first fault: H.13.2.2.2. Evaluation: H.13.2.5. Tolerance and drift: H.17. Measures per MCU part: Table H.2.

## Decision

### Framing

- **The architecture follows IEC 62109-1 Annex B route (a).** Each protection must work with the CPU dead or the software disabled, and it is tested that way.
- **The process follows route (b).** Our hardware protection is configured and locked by firmware, so that firmware, the CPU supervision and the self-tests follow IEC 60730-1 Annex H, **Class B**, as single channel with functional test plus periodic self-tests. Class C would need dual-channel hardware the Spin does not have.

### Protection paths

| Path | Mechanism | Declared fault reaction time | Needs the CPU? |
|---|---|---|---|
| **Comparator path** | COMPx → HRTIM FLTx; HRTIM digital filter about 300 ns against switching spikes | < 0.5 µs | No, once armed and locked |
| **ADC-watchdog path** | ADC analog watchdog → EEVx → FLTx; fault counter of 2 events; OVRMOD = 1 | ≤ 3 switching periods (60 µs at 50 kHz) | No, while the HRTIM keeps triggering the ADC |
| **CPU supervision** | IWDG fed only from the control task (1 ms); a fatal-error handler (override of `k_sys_fatal_error_handler`) that disables every HRTIM output and drives the safe outputs before halting; system fault enabled (lockup, PVD, clock security, ECC) | Watchdog 1 ms; handler and system fault < 10 µs | Covers the CPU itself |
| **Software path** | Today's polling, kept for what the hardware cannot watch | 5 control periods (250 µs) | Yes |

### One Safety API

- The user declares a limit per sensor, or per computed quantity. **The build picks the strongest path available**: comparator, then ADC watchdog, then software. The user never chooses a path.
- The software path remains necessary for three reasons:
  - The hardware runs out of channels: 6 fault inputs, and only ADC1/ADC2 watchdogs reach them.
  - Some quantities must be computed: RMS, frequency, grid window, synchronization, power, temperature derating.
  - Plausibility checks and self-tests are software.
- The old functions stay as compatibility wrappers.
  - `setChannelReaction()` is replaced by the per-leg safe state.
  - NVS-stored thresholds are accepted only when tighter than the devicetree limit.
  - `disableSafetyApi()` affects only the software path, and only in development mode.

### Configuration

- **Limits** are devicetree values in physical units: the board's absolute maximum, set by the shield author.
  - User code may **tighten** them at run time with `safety.setTripThreshold()`, never loosen them.
  - DAC references and ADC watchdog thresholds are **read back every period**. The read-back trips on any mismatch.
- **Comparator reference**: a DAC, written once from the devicetree value and then read back. VREFINT fractions rarely match a useful threshold.
- **Fault-channel allocation is computed at build time** from the shield devicetree. A required protection with no free channel is a **build error**, unless the shield or the `app.overlay` declares the exception explicitly.
- **Bidirectional currents on today's boards**: the comparator takes one sign (the default is the conduction direction of the leg's high-side switch) and the ADC watchdog takes the other. A shield property overrides the default.
- **Peak-current mode**: the peak reference is clamped below the trip, and that leg trips through its ADC watchdog.
- **Protection manifest**: the build writes `protection-manifest.json`, from Zephyr's parsed devicetree (`edt.pickle`) and the allocation. It is the **single source** for:
  - the boot line;
  - the docs protection table;
  - the test firmware's expectations;
  - every generated test case.

### Behaviour

- **Arming**: `shield.power.start()` arms automatically if `safety.arm()` was not called. Arming means:
  1. configure;
  2. **functional test of every hardware trip**: with the outputs held off, move each threshold past the live reading, check that the fault flag sets, restore;
  3. **lock** (FLTxLCK, FLTLCK, COMPx LOCK).

  Any failure means no PWM.
- **Every trip stops every leg.** Each leg goes to **its own safe state**: `safe-state = "open" | "short-low" | "short-high"` on the leg node, or `safety.setSafeState()` before the PWM starts. The default is `"open"`. A `short-*` leg requires a trip on its own low-side current, or a devicetree statement accepting the risk.
- **Trips latch** until `safety.clearFault()`, which works only once the source is quiet and the PWM is stopped. Nothing retries automatically, and a repeated clear within N s needs a reboot. A clear with the fault still present trips again into the same state (H.13.2.2.2 b).
- **Fault interrupt**: a zero-latency interrupt, above Zephyr's. It drives the **safe outputs** and records {source, path, sensor, time, last ADC values} for `safety.getFault()`.
- **Safe outputs** are declared in the shield devicetree: GPIO, safe level, and `reset-state-safe;` once the shield author has checked the board's pull resistor. Examples are the grid relay and driver enables. A missing `reset-state-safe;` gives a build warning that explains why (after a watchdog reset the pins float) and how to fix it. An app adds its own safe outputs with one line in `app.overlay`.

### Development mode

- `safety-mode = "development"` in `app.overlay`. Only then do `safety.disarm()` and run-time loosening **compile**: in any other build the calls are a compile error.
- Development mode prints a boot banner, sets a telemetry flag and tags the version string.
- **CPU supervision is never optional**, not even in development mode.

### Class B self-tests (IEC 60730-1 Table H.2)

- **At start-up**, always:
  - CPU register patterns;
  - March C over RAM;
  - flash image CRC (hardware CRC unit);
  - main clock against the LSI;
  - the trip functional test above.
- **At run time**, in background slices. Kconfig `CONFIG_OWNTECH_SAFETY_RUNTIME_SELFTEST`, on by default, can be turned off only in development mode:
  - program-flow checkpoints gating the watchdog feed;
  - interrupt rate against an independent timer;
  - VREFINT conversion range;
  - RAM slices, with the safety data kept as value plus inverted copy;
  - flash CRC slices;
  - CPU registers.
- **Sensor plausibility** (generic): the offset at idle within a band, and a reading frozen for N periods while the PWM runs. Cross-checks between sensors stay board-specific and optional.
- The self-tests are written in OwnTech code, following ST's Class B methods; X-CUBE-CLASSB is ST-licensed and not built for Zephyr. They live in a module under `zephyr/modules/`. Zephyr is not patched: the fatal hook is a supported weak override.

### Teaching and examples

- Examples gain **no safety calls**.
- The board shows one boot line, e.g. `safety: armed I1_LOW comp+, I2_LOW comp+, I_HIGH awd± · self-test OK`.
- Trip messages name the sensor, the path, the value and how to clear.
- A per-board protection table is generated into the docs from the manifest.

### Tests

All tests are generated from the protection manifest, with one hand-written input per board: its **bench file** (where the fault switch goes, source limits, output ports, scope wiring).

- **Generic test firmware.** One ThingSet app built for any shield, on the model of SPOC's `Core-CUT`. It arms, runs the PWM, reports and clears faults, and injects software faults:
  - stack overflow;
  - hard fault;
  - interrupts masked;
  - watchdog starved;
  - wild write to the DAC/COMP/HRTIM registers after locking.
- **L0 (bare Spin on the SPOC bench, Rigol MHO84):**
  - one case per (limit × sign × path), plus CPU-supervision cases;
  - edge cases: just below the threshold; a glitch shorter and one longer than the filter; a fault while arming; a fault during clear; two faults at once; a fault during dead time.
  - Injection, self-injection: CH1 selects a CUT DAC pin, CH2 the target, and one wire joins test points TP1 and TP2 (before the 9 MΩ dividers).
  - Injection, independent: a third "injector" Spin wired to a test point, for faults while the CUT's CPU is dead.
  - Routes are set with the PWM stopped.
  - Each campaign starts with a **skew and gain calibration**: one CUT pin routed to all four channels.
  - The comparator latency verdict (31 ns) uses direct probes. The SPOC mux path adds an estimated tens of ns.
- **L1 (switched model):** the protection hardware is modelled in the plant (comparator with 31 ns, latch, per-output fault state, ADC watchdog at the ADC trigger) from the same manifest. Scenarios:
  - load short;
  - shoot-through;
  - DC bus overvoltage;
  - grid surge;
  - a sensor stuck at zero;
  - CPU death mid-period.
- **L2 (powered)**, criteria generated for the existing bench app, following the norms. For each protection, at full and at limited source current:
  - software path disabled (62109-1 B.1.1 a);
  - a CPU-dead variant;
  - the fault at random PWM/control phases and Spin supply at 85 % and 110 % (60730-1 H.13.2.5.1);
  - 3 immediate re-clears (62109-1 4.4.2.2);
  - single component faults: sensor open, stuck PWM line, bridge switch shorted (62109-1 4.4.4.1);
  - output shorts on every terminal pair and all together, with peak and 3-cycle RMS current recorded (4.4.4.5);
  - output overload (4.4.4.7);
  - backfeed with the bridge stopped (4.4.4.6).

  Three Spins measure the trip value and time spread, before and after thermal cycling and endurance (60730-1 H.17). Powered tests start at 50 V behind the series bulb and step up to rated voltage case by case.
- **Scope**: one interface, with the Rigol MHO84 as the first backend.
- **Coverage**:
  - every campaign build-checks every shield devicetree (Twist 1.2.0–1.4.2, Ownverter 0.9.0–1.1.0, uSolarVerter 0.9.0, O2 1.1.2);
  - on-target L0 runs for the newest board of each family;
  - L2 runs per board as its bench file is written.
- **New boards** follow `docs/safety_new_board.md`, including the design-time check `tools/safety_check.py <draft.overlay>`. It runs the same allocation on a draft devicetree and prints the coverage report before layout; the report is attached to the schematic review.

### Shield design rules (future boards)

- Every switch current and the AC current go to **two** comparator inputs, one per sign, where the pins allow it. Spare Spin pins: PB1 (37), PA3 (52), PA7 (44); PB0 (31) and PB11 (1) where free.
- Gate-driver FAULT/DESAT outputs, or a shield window comparator, go to an **HRTIM fault pin**: PA15 (48), PB10 (41), PC10 (16) or PC7 (9).
- Safe outputs and driver enables are pulled to their safe level, so that they are safe while the MCU resets.

For the uSolarVerter: owntech-foundation/micro-inverter#48.

### Out of scope

EMC immunity (IEC 60730-1 H.25, level 3 for a protective control) is left for its own series of runs.

## Consequences

- A board whose limits cannot all be covered no longer builds silently. Legacy shields need explicit exception lines in their devicetree for the sensors with no hardware path.
- Existing apps keep running. The devicetree defaults are the boards' absolute ratings, and the protection is on by default.
- About 6–8 KB more flash (the USB image slot is 222 KB, 76 % used) and about 20 ms more boot. The control task gains a few instructions per period: the watchdog feed, a flow checkpoint, the VREFINT check.
- A locked comparator or fault set-up can change only after a reset. Run-time tightening goes through DAC and ADC-watchdog thresholds with read-back.
- A watchdog reset reboots the board in the middle of a session, which loses the console and RAM. That is accepted: it is the only exit from a CPU that is stuck while still running.
- The test infrastructure of `apf_quadrature` (record writer, catalogue, control center) is brought board-agnostic onto this branch so the safety tests can show in the control center. That infrastructure goes upstream with it.

## Proposal pending team review: fault-switch fixture

IEC 62109-1 4.4.4.1 asks for faults applied "using a switching device" with low impedance while the converter runs. The proposed fixture:
- a power MOSFET pair connected back to back (bidirectional for AC), with its own isolated driver;
- placed across the AC output, the DC bus, or one bridge switch;
- closed by the injector Spin at a programmed or random phase of the PWM.

It makes "fault at any point of the program sequence" (60730-1 H.13.2.5.1) repeatable and keeps the operator's hands away from short-circuit tests. Ratings, layout and the current-limited first runs are to be agreed with the hardware team before anything is built.

## Implementation notes

**2026-11-14, CPU supervision (luizvilla/Core#16), measured on a bare Spin (L0-SUP).** Zephyr formats and logs the exception and the fatal error before it calls `k_sys_fatal_error_handler()`: 250 to 300 µs with the default `CONFIG_EXCEPTION_DEBUG=y`, during which the PWM kept switching. Overriding the handler is therefore not enough. CPU supervision wraps the two C entry points of the fault path with the linker (`--wrap=z_arm_fault`, `--wrap=z_fatal_error`) and disables every HRTIM output first: 0.82 µs from a stack overflow, 0.95 µs from an undefined instruction. Consequences: any CPU fault stops the PWM, including faults Zephyr could recover from (userspace, debug monitor; unused by OwnTech); a Zephyr update that renames these functions makes the link fail instead of losing the protection silently. The watchdog timeout follows the control task: at least 3 periods, never under the 1 ms default (6.9 ms with a 2 ms task). A fault caught by CPU supervision stays latched across resets (TAMP backup registers) until `safety.clearFault()` or a power cycle: nothing restarts the converter on its own.
