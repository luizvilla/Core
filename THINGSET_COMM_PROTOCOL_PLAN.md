# Extend Python and MATLAB ThingSet Clients

## Summary

Add persistent writable converter metadata, matched Python/MATLAB
power-test-bench wrappers, safe examples, tests, and documentation. Preserve
the generic ThingSet APIs and require explicit opt-in before energizing
hardware.

## Functional Coverage Compared with `old4`

This comparison uses `old/old4/comm_protocol.cpp` as the legacy reference and
the current branch after the dual-serial scope implementation (including the
rebased Power API change `aab0ae5`) as the ThingSet implementation. "Present"
means that an equivalent
application behavior is exposed by the current firmware and, where relevant,
by both the Python and MATLAB wrappers. "Partial" means that an object or API
exists but behavior, validation, reporting, or hardware execution is
incomplete. "Missing" means that neither the current application object tree
nor the wrappers expose the legacy behavior. "Replaced" means that the
capability exists through ThingSet but the old wire format is intentionally not
compatible.

| `old4` capability | Legacy implementation | Current ThingSet implementation | Python/MATLAB client coverage | Status and remaining gap |
|---|---|---|---|---|
| Serial command parsing and dispatch | `initial_handle()`, `console_read_line()`, `defaultHandler()`, `powerLegSettingsHandler()`, and single-letter/underscore commands | Zephyr ThingSet shell with typed paths, standard responses, and object discovery | Generic `get`/`read`/`write`/`discover` APIs in both clients | **Replaced** — equivalent transport is present, but legacy commands such as `d_i` and `s_LEG1_d_0.1` are not accepted |
| Tester states | `_i`, `_f`, and `_o` select `IDLE`, `POWER_OFF`, and `POWER_ON` | `/Config/Mode` exposes the same three states | `set_mode`/`setMode`; safe sequencing in `power_on`/`powerOn` and `shutdown` | **Present** |
| Leg selection | Commands address Leg 1, Leg 2, and conditionally Leg 3 | `/Config/Leg1` and `/Config/Leg2`; firmware is compiled for two TWIST legs | Both wrappers accept only Leg 1 or Leg 2 | **Partial** — TWIST parity is present; conditional Ownverter Leg 3 is missing |
| Per-leg enable | `_l_on` and `_l_off` update the leg-on setting | `/Config/LegN/wEnable` starts/stops a leg together with global mode | `configure_leg`/`configureLeg` | **Present** |
| Capacitor switch | `_c_on` and `_c_off` call the TWIST capacitor API | `/Config/LegN/wCapa` callback connects or disconnects the capacitor | Per-leg configuration plus explicit power-on options | **Present** |
| Driver switch | `_v_on` and `_v_off` call the driver API | `/Config/LegN/wDriver` callback calls `connectDriver()` or `disconnectDriver()` | Per-leg configuration and optional shutdown hardware disconnect | **Partial / defective** — connect works, but current `PowerAPI::disconnectDriver()` calls `setPin()` instead of clearing the pin |
| Buck/boost selection | `_b_*` and `_t_*` store buck and boost flags | `/Config/LegN/wBuck` and `wBoost` select PID duty and boost inversion | Wrappers enforce mutually exclusive buck/boost selection | **Present** |
| Manual duty cycle | `_d_` accepts values from 0 to 1 | `/Config/LegN/wDutyCycle` with firmware restore on out-of-range writes | Range validation and readback verification in both wrappers | **Present** |
| Reference and tracking variable | `_r_<channel>_<value>` selects `V1`, `V2`, `VH`, `I1`, `I2`, or `IH`; Ownverter also adds `V3` and `I3` | `/Config/LegN/wReferenceValue` and `wTrackingVar` support the six TWIST channels | Channel-name validation and readback verification | **Present for TWIST; missing for Ownverter V3/I3** |
| Phase shift | `_p_` validates against legacy minimum/maximum values and applies immediately | `/Config/LegN/wPhaseShift` validates `-360` to `360` and applies immediately | Same range enforced by both wrappers | **Present** |
| Rising and falling dead time | `_x_` and `_z_` validate against legacy minimum/maximum values and apply immediately | `/Config/LegN/wDeadTimeRising_ns` and `wDeadTimeFalling_ns` apply immediately | Both wrappers validate only the `uint16` range | **Partial** — setting is present, but hardware-specific minimum/maximum validation is not |
| Switching frequency | `_f_` checks the HRTIM minimum and calls the low-level PWM setter | `/Config/Frequency_Hz` and wrapper methods exist | `set_frequency`/`setFrequency` writes and verifies readback | **Partial / defective** — the callback still compares against an uninitialized zero maximum and bypasses the rebased `shield.power.setFrequency()` bookkeeping, so valid changes such as 100 kHz are restored |
| Sensor calibration | `calibrationHandler()` sets gain/offset, stores them, retrieves them, and prints persistence status | `/Calibration/<channel>/{wGain,wOffset,wStore}` sets values and performs store/retrieve | `set_calibration`/`setCalibration` and read methods | **Partial** — calibration and persistence are present, but firmware does not expose NVS store/retrieve success or restore values on persistence failure |
| Power-off configuration frame | `frame_POWER_OFF()` prints factory-test flags and every leg's state, duty, reference, and tracking selection in one line | `/Config`, `/Config/LegN`, and `/Measurements` provide typed on-demand reads | Generic reads plus `read_leg`/`readLeg` | **Partial / replaced** — converter configuration is available, but there is no single legacy frame and no RS485/sync/analog/CAN result block |
| Power-on telemetry frame | `frame_POWER_ON()` prints duty, voltages, currents, peaks, temperatures, high-side values, and factory-test fields | `/Measurements` exposes V1/V2/VH, I1/I2/IH, both temperatures, V1/V2 peaks, and both duty readbacks | `read_measurements`/`readMeasurements` | **Partial / replaced** — TWIST converter telemetry is present; the diagnostic tail and optional Leg 3 telemetry are missing |
| Automatic/push telemetry | Legacy code can emit fixed-format power-on and power-off lines without a structured request | Current serial API is request/read based; no application telemetry stream or report schedule is defined for these objects | Wrappers poll measurements explicitly | **Missing** if legacy push/stream behavior is required |
| Scope acquisition toggle | `scopeHandler()` toggles `enable_acq` through `_a` | `/Debug/Scope` provides validated one-shot arm/trigger, state/error readbacks, pre-trigger ratio, and decimation 1–100 | `arm_scope`/`armScope`, `trigger_scope`/`triggerScope`, and duration-aware ready waits | **Partial pending hardware acceptance** — firmware and fake-client gates pass; live state/timing gates await recovery of the target board |
| Scope buffer download | `scopeHandler()` sets `is_downloading` through `_r` | Dedicated `if00` `D` command emits the complete frozen 1,024-by-8 buffer in legacy text/hex while ThingSet remains on `if02` | Explicit-port Python/MATLAB `ScopeSerial` parsers and `download_scope`/`downloadScope` | **Partial pending hardware acceptance** — bounded parsing, rotation, malformed-input handling, and wrapper parity pass offline; live transfer/repeated-download parity awaits target recovery |
| RS485 slave response | `slave_reception_function()` echoes and increments the RS485 test data, adds sync/CAN/analog fields, and starts transmission | The repository contains communication drivers, but this application does not configure or expose this factory-test transaction | No wrapper method | **Missing** |
| RS485/CAN/sync/analog master verification | `master_reception_function()` evaluates the returned fields and latches `RS485_success`, `Sync_success`, `Analog_success`, and `Can_success` | No application state, callback, or ThingSet object implements this diagnostic workflow | No wrapper method | **Missing** |
| Conditional Ownverter support | Compile-time Leg 3, V3/I3 tracking, V3/I3/temperature/peak telemetry | `POWER_LEG_COUNT` is 2 and `CALIBRATION_CHANNEL_COUNT` is 6; object IDs are explicitly defined for two legs | Both wrappers reject Leg 3 and V3/I3 | **Missing** |
| Runtime PID sensitivity tuning | Handler tables and PID parameter setters are commented out in `old4` | Current PID parameters remain compiled constants | No wrapper method | **Not an active `old4` baseline feature**; still missing if the commented experiment is revived |
| Converter metadata | No equivalent in `comm_protocol.cpp` | Persistent `/Converter` board name, board version, serial number, and firmware version | Read/write metadata methods with validation | **Added in current implementation** |
| Self-description and verified writes | Fixed parser and positional frames | ThingSet discovery, typed objects, access metadata, status responses, and post-write callbacks | Both clients discover objects and wrappers read back every safety-relevant write | **Added in current implementation** |
| Failure-safe client sequencing | Host was responsible for ordering legacy commands | Firmware separates global mode from per-leg enable; wrappers request `POWER_OFF`, configure one leg, then request `POWER_ON`, with cleanup on failure | Implemented equivalently in Python and MATLAB | **Added in current implementation** |

The commented PID-sensitivity handlers in `old4` are deliberately excluded
from required parity. The communication drivers available elsewhere in the
repository do not count as current application coverage unless this firmware
registers the corresponding objects/callbacks and the host wrappers expose the
workflow.

If full `old4` parity is required, the priority order is:

1. Correct frequency validation and route accepted writes through
   `shield.power.setFrequency()`.
2. Correct `PowerAPI::disconnectDriver()` and verify physical driver-off
   readback or behavior.
3. Complete live timing, repeated-download, and cross-language acceptance for
   the implemented dual-serial scope after the TWIST target is recovered.
4. Decide whether the RS485/CAN/sync/analog factory-test workflow belongs in
   this application; if so, expose its commands, status, and failure details as
   typed objects.
5. Add a report/streaming mechanism only if polling `/Measurements` is
   insufficient.
6. Generalize the object tree and both wrappers before claiming Ownverter
   Leg 3 parity.

## Firmware and Metadata

- Add an application-owned `/Converter` group, avoiding the CAN module's
  existing read-only `/Device` registration.
- Register four `THINGSET_ANY_RW` strings:
  - `/Converter/wBoardName`
  - `/Converter/wBoardVersion`
  - `/Converter/wSerialNumber`
  - `/Converter/wFirmwareVersion`
- Store them in `converter_metadata_t`, with capacities of 24, 16, 48, and 32
  bytes. Defaults are the devicetree shield name/version (`TWIST`, `v1.4.2`),
  `"UNSET"`, and `"1.0.0"`.
- Add `converter_metadata_cb` using PRE_WRITE/POST_WRITE snapshots. Accept
  non-empty printable strings and restore the previous complete structure on
  invalid input or persistence failure.
- Persist the complete structure under application-owned NVS key `0x0401`.
  Load it during startup, retaining compiled defaults when stored data is
  absent or invalid.
- Document `/Device` as framework-owned/read-only and `/Converter` as
  application-owned/writable/persistent.

## Client Interfaces

- Add exact path-aware access overrides for `/Config/Mode` and
  `/Config/Frequency_Hz` in both generic clients while preserving their public
  APIs.
- Add a separate Python `PowerTestBench` wrapper with `TesterMode`,
  `PowerTestBenchError`, mode/frequency/leg/calibration/metadata accessors,
  `power_on`, and `shutdown`.
- Add a MATLAB `PowerTestBench` class with equivalent camelCase methods.
- Validate numeric ranges, supported leg/channel/tracking names, mutually
  exclusive buck/boost modes, and metadata length/printability.
- Read back writes and report values rejected or restored by firmware.
- `power_on` requests `POWER_OFF`, applies settings and enables, then requests
  `POWER_ON`; partial failure triggers shutdown.
- `shutdown` requests `POWER_OFF` first and clears both leg enables.
  Capacitors and drivers remain unchanged unless explicitly requested.

## Examples, Tests, and Documentation

- Make the Python example a CLI with port, leg, duty, duration, and explicit
  power/driver/capacitor flags.
- Make the MATLAB example a callable function with equivalent name-value
  options and `onCleanup`.
- Safe defaults discover objects, read metadata and measurements, enforce
  `POWER_OFF`, and leave both legs disabled.
- Add Python `unittest` and MATLAB `matlab.unittest` coverage using fake
  clients for classification, validation, mapping, metadata, sequencing,
  readback rejection, and failure cleanup.
- Validate with Python compilation/tests, MATLAB `checkcode`/tests, and
  `pio run -e USB`.
- Hardware acceptance covers metadata write/reboot persistence, writable
  Mode/Frequency, safe defaults, explicit power activation, and interruption
  cleanup.
- Update both READMEs with the new tree, persistence behavior, APIs, examples,
  and corrected access classification.

## Commit Sequence

0. `docs: add extended ThingSet client implementation plan`
   - Replace the existing contents of `THINGSET_COMM_PROTOCOL_PLAN.md` with
     this decision-complete plan, including the metadata design, tests, safety
     behavior, and full commit sequence.
   - Stage and commit only the plan file so implementation starts from a
     documented baseline.

1. `feat: add persistent converter metadata objects`
   - Add `/Converter`, its storage structure, callback validation, NVS
     persistence/loading, and startup initialization.
   - Gate: `pio run -e USB` passes.

2. `fix: support path-aware ThingSet write access`
   - Correct Python and MATLAB discovery for writable fields without `w`
     prefixes and add classification tests.
   - Gate: existing generic APIs remain compatible and tests pass.

3. `feat: add Python power test bench interface`
   - Add the Python wrapper, metadata methods, safety sequencing, fake client,
     and unit tests.
   - Gate: Python compilation and unit tests pass.

4. `feat: add MATLAB power test bench interface`
   - Add the matched MATLAB wrapper, metadata methods, fake client, and tests.
   - Gate: MATLAB static analysis and unit tests pass.

5. `docs: update ThingSet power test bench examples`
   - Convert both examples to safe opt-in operation and commit the existing
     root/source README changes with the final interface documentation.
   - Gate: all automated checks and the firmware build pass.

## Assumptions

- Metadata persists automatically after every successful write.
- `/Converter` is used because redefining `/Device` would collide with the
  enabled CAN module.
- Python and MATLAB wrappers remain separate from their generic transport
  clients.
- No reference-sweep client is included. Scope data retrieval is planned
  below, while automatic plotting remains outside the Python and MATLAB
  wrappers.
- The exact tracked filename is `THINGSET_COMM_PROTOCOL_PLAN.md`.

---

# Dual-Serial Scope Feature Implementation

## Summary

Add a resumable scope implementation in which ThingSet controls acquisition
through the dedicated shell interface (`if02`) and capture data is downloaded
exclusively through the board's non-ThingSet console interface (`if00`).

The fixed design is:

- 1,024 samples and eight channels.
- A 100 us critical-task period and user-configurable decimation from 1 to
  100.
- Effective sample period of `100 us * decimation`.
- Capture duration of `102.4 ms * decimation`, from 102.4 ms to 10.24 s.
- Default decimation of 1 and default pre-trigger ratio of 20%.
- Channels V1, V2, VHigh, I1, I2, IHigh, Duty1, and Duty2.
- Legacy-compatible text/hex downloads.
- Equivalent Python and MATLAB scope APIs using a separate, explicitly
  selected data port.
- No scope-port auto-detection, frequency writes, or power-supply control
  during acceptance.

The two scope entries in the old4 comparison above are implemented and remain
**Partial pending hardware acceptance** until the blocked Phase 2/3 live gates
and final Phase 6 acceptance are complete.

## Public Interface and Protocol

Add `/Debug/Scope` with unique ThingSet IDs:

| Object | Behavior |
|---|---|
| `wArm` | One-shot boolean that resets and arms acquisition, then returns to `false` |
| `wTrigger` | One-shot software trigger, valid only while armed, then returns to `false` |
| `wPretriggerRatio` | Configuration for the next capture; float, default 0.2, range 0.0-0.9 |
| `wDecimation` | Configuration for the next capture; integer, default 1, range 1-100 |
| `rState` | `IDLE`, `ARMED`, `TRIGGERED`, `READY`, `STREAMING`, or `ERROR` |
| `rSampleCount` | Constant 1024 |
| `rChannelCount` | Constant 8 |
| `rSamplePeriod_us` | Period latched at arm: `100 * wDecimation` |
| `rCaptureDuration_ms` | Duration latched at arm: `102.4 * wDecimation` |
| `rFinalIndex` | Circular-buffer final index |
| `rLastError` | `NONE`, `INVALID_STATE`, `INVALID_DECIMATION`, `TRANSFER`, or `INTERNAL` |

| Decimation | Scope call frequency | Sample period | Duration |
|---:|---|---:|---:|
| 1 | Every critical iteration | 100 us | 102.4 ms |
| 10 | Every tenth critical iteration | 1 ms | 1.024 s |
| 100 | Every hundredth critical iteration | 10 ms | 10.24 s |

`wDecimation` and `wPretriggerRatio` configure the next arm operation. Writes
during `ARMED`, `TRIGGERED`, or `STREAMING` are restored and reported as
invalid-state errors. Writes in `IDLE` or `READY` are allowed. Effective
period and duration readbacks retain the frozen capture's settings until the
next arm.

The critical task resets a decimation counter at arm and calls
`scope.acquire()` once every configured number of critical iterations. A
software trigger is latched until the next acquisition tick, giving a maximum
trigger alignment latency of one effective sample period.

Callbacks validate values and queue requests only. The critical task owns
ScopeMimicry acquisition and the background data-port task owns serial I/O.

The permanent 115200-baud data-port commands are:

- `?` returns `SCOPE-DATA/1 OK`.
- `D` downloads a frozen capture in `READY`.
- An unknown byte returns `SCOPE-DATA/1 ERROR UNKNOWN_COMMAND`.
- `D` outside `READY` returns `SCOPE-DATA/1 ERROR NOT_READY <state>`.

The data interface must never be opened at 1200 baud because that rate invokes
the USB bootloader callback.

The compatible download format is:

```text
begin record
#V1Low_V,V2Low_V,VHigh_V,I1Low_A,I2Low_A,IHigh_A,Duty1,Duty2,
# <final-index>
<8192 lines containing one 8-digit IEEE-754 hexadecimal value>
end record
```

Values are sample-major and channel-interleaved. Hosts require the exact count
and rotate rows beginning at `(finalIndex + 1) % 1024` into chronological
order. Firmware serializes the complete ScopeMimicry buffer using `memcpy` to
`uint32_t`; it does not use the library's `dump_datas()` implementation, which
omits its final value.

Python adds `ScopeSerial`, `ScopeSerialError`, and `ScopeCapture`, and extends
the backward-compatible `PowerTestBench(client, scope_client=None)` with
`read_scope_status`, `arm_scope`, `trigger_scope`, `wait_scope_ready`, and
`download_scope`.

MATLAB adds a matched `ScopeSerial` handle class and extends
`PowerTestBench(client, scopeClient)` with `readScopeStatus`, `armScope`,
`triggerScope`, `waitScopeReady`, and `downloadScope`.

Both arm APIs accept pre-trigger ratio and decimation. Capture results contain
the active decimation, effective period, duration, channel names,
chronological samples, final index, pre-trigger ratio, and trigger-relative
time axis. Default ready timeouts are calculated from capture timing with a
two-second margin. `ScopeSerial` always requires an explicit port and probes
it before use.

## Progress Ledger and Session Handoff

| Phase | Status | Baseline / result | Gate and evidence | Next action |
|---:|---|---|---|---|
| 0. Plan and global convention | PASS | Commit `a6ca0f7` | Global rule created; staged-file gate passed | Complete |
| 1. Data-port viability | PASS | Based on `a6ca0f7` | Build 41.4% RAM; upload selected `5843500300470047`; 3 probes, unknown command, `/Converter`, and isolation passed | Complete |
| 1A. Upload isolation remediation | PASS | Commits `a47a096`, `88d7ba0` | Exact board ID is mandatory; uploader waits for the selected serial to identify as MCUboot and stores its stable by-id path; Python compilation passed | Complete |
| 2. Decimated acquisition | BLOCKED | Commit `feat: add decimated ThingSet scope acquisition` | Clean build passed at 41.7% reported RAM with 4 KiB system heap; hardware state gate awaits a physical TWIST reset | Rerun hardware state gate after target recovery |
| 3. Bounded download | BLOCKED | Commit `feat: stream scope captures on the data serial port` | Build passed at 41.7% RAM; source emits exact header, all 32,768 buffer bytes as 8,192 hex lines, and returns `STREAMING` to `READY`; live download gate awaits target recovery | Rerun hardware transfer gate after target recovery |
| 4. Python client | PASS | Commit `feat: add Python decimated scope capture interface` | `compileall` and 23 `unittest` cases pass, including bounded parsing, rotation, malformed input, scope validation, readback rejection, sequencing, and missing transport | Complete |
| 5. MATLAB client | PASS | Commit `feat: add MATLAB decimated scope capture interface` | `checkcode` reports zero issues and all 19 `matlab.unittest` cases pass, including matched parser, scope validation, sequencing, and transport cleanup | Complete |
| 6. Documentation and acceptance | BLOCKED | Commit `docs: document and validate decimated scope capture` | Both READMEs and `automated_test.md` updated; 23 Python tests, 19 MATLAB tests with zero `checkcode` findings, and USB build at 41.7% RAM/81.6% flash pass; live acceptance awaits physical target reset | Recover TWIST, rerun Phase 2/3 live gates, then mark scope parity present |

Only one phase may be `IN_PROGRESS`. Before and after each phase, update its
status and record command results, identity evidence, incidents, recovery
actions, and the next command. Never erase a failed attempt. A new session
must inspect this ledger and the worktree, verify USB identities, and rerun
the previous phase's gate before proceeding.

Incident log:

1. `2026-07-29` — A read-only fresh Codex CLI verification was attempted
   three times. The installed Codex 0.92.0 CLI rejected the configured
   `gpt-5.6-sol` model as requiring a newer CLI and rejected the two explicit
   fallback models for ChatGPT-account compatibility. The instruction file was
   verified directly at `/home/luiz-villa/.codex/AGENTS.md`; upgrading the CLI
   is outside this repository's scope.
2. `2026-07-29` — The first data-port probe read the configured delayed-boot
   banner instead of the application acknowledgement. After allowing the
   1.5-second startup delay and draining console text, three consecutive `?`
   probes returned `SCOPE-DATA/1 OK`. The retry also verified the unknown
   command response, `/Converter` on `if02`, and zero cross-port bytes.
3. `2026-07-29` — The first Phase 2 build failed because Zephyr's selected
   minimal C++ library does not provide `<cmath>` or `<limits>`. The
   implementation was changed to the equivalent C `math.h`, `stdint.h`,
   `isfinite`, and `UINT16_MAX` interfaces before retrying.
4. `2026-07-29` — After the Phase 2 upload explicitly selected and reset
   TWIST serial `5843500300470047`, the protected OWNVERTER serial
   `584350030047002D` enumerated as `MCUBOOT` instead of
   `OWNVERTER_V1_1_0`. No command in this test targeted or opened its port.
   Testing continued only against the already-flashed TWIST; further uploads
   were suspended pending recovery and a final isolation review.
5. `2026-07-29` — The first Phase 2 image enumerated but did not reach its
   application tasks. Linker and libc inspection showed that ScopeMimicry's
   `new[]` uses newlib `malloc`, not Zephyr's system heap. Raising
   `CONFIG_HEAP_MEM_POOL_SIZE` to 40 KiB left only about 33 KiB for newlib,
   which was insufficient for the 32,768-byte buffer plus allocator
   overhead. The override was removed so the existing 4 KiB system heap
   leaves roughly 70 KiB available to newlib.
6. `2026-07-29` — Upload-script inspection identified the isolation race.
   After the 1200-baud request, the script accepted the still-enumerated
   application port instead of waiting for the selected serial to enumerate
   as `MCUBOOT`. The saved `/dev/ttyACM*` connection could then be rebound to
   another board. The uploader now aborts if an explicit board ID is absent,
   selects the application console interface, waits for both the selected
   serial and `MCUBOOT` product, and stores the corresponding serial-specific
   `/dev/serial/by-id` path before configuring `mcumgr`. OWNVERTER returned to
   `OWNVERTER_V1_1_0` without being opened, reset, or probed by this test.
   The TWIST application remained wedged, and its 1200-baud callback could not
   be reached; its hardware gate therefore awaits a physical target reset.
7. `2026-07-29` — The first MATLAB parser test run failed because the fake
   transport converted double-quoted `sprintf` strings directly to `uint8`,
   which MATLAB rejects. The fixtures were changed to character output; the
   rerun passed all 19 tests with no `checkcode` findings.

## Phases and Commit Sequence

### Phase 0 — Persist the plan and convention

- Preserve the existing plan and old4 comparison and append this extension.
- Store the permanent commit-sequence rule in
  `/home/luiz-villa/.codex/AGENTS.md`; it is local configuration and not a
  Core4 commit.
- Preserve and leave unstaged the unrelated `src/app.overlay` change.

Commit 0: `docs: add dual-serial scope implementation plan`

Gate: only `THINGSET_COMM_PROTOCOL_PLAN.md` is staged.

### Phase 1 — Prove the non-ThingSet serial path

- Add a minimal background component on `zephyr,console`
  (`cdc_acm_uart0`).
- Implement the permanent `?` probe and unknown-command response.
- Build and flash only TWIST, send `?` to `if00` three times, and require the
  exact response.
- Confirm ThingSet `/Converter` reads still work on `if02`, with no traffic
  crossing between the interfaces.
- Mark the phase `BLOCKED` and stop if identity, probe, or isolation fails.

Commit 1: `feat: identify the scope data serial port`

### Phase 1A — Remediate upload isolation

- Treat a configured `board_id` as mandatory and abort when it is absent.
- Select the application console interface for the 1200-baud request.
- Do not configure `mcumgr` until the selected USB serial identifies itself
  with product `MCUBOOT`; never retain a stale application `/dev/ttyACM*`.
- Keep this remediation independent of the scope implementation so future
  multi-board uploads inherit the isolation fix.

Commit 1A: `fix: pin USB uploads to selected bootloader identity`

Gate: the uploader script compiles, explicit-target fallback is absent, and
the MCUboot wait requires both the selected serial and product.

Follow-up commit 1B: `fix: keep mcumgr uploads on stable serial paths`

Gate: after identity verification, Linux `mcumgr` profiles use the matching
serial-specific `/dev/serial/by-id` link instead of a reusable tty number.

### Phase 2 — Implement decimated acquisition

- Pin ScopeMimicry to
  `6f49b722fa508703387aa87ed95d75d1044a8f06`.
- Keep Zephyr's existing 4 KiB system heap and assert that the 32,768-byte
  buffer fits the library size API. ScopeMimicry's C++ `new[]` uses the
  separate newlib heap in otherwise-unused RAM after static allocation.
- Connect the eight channels and add scope state, configuration validation,
  active timing snapshots, the decimation counter, and the latched trigger.
- Call `acquire()` at the end of selected critical iterations after
  measurements and duty updates.
- Keep acquisition available in `POWER_OFF`.

Commit 2: `feat: add decimated ThingSet scope acquisition`

Gate: build and RAM pass; the complete scope tree is discovered; decimations
1, 10, and 100 report 100, 1000, and 10000 us; 0 and 101 are restored; active
configuration writes are rejected; and arm/trigger/re-arm transitions pass.

### Phase 3 — Implement bounded download

- Add `D` and emit all 8,192 values in the legacy format from a background
  task.
- Reject requests outside `READY`, use `STREAMING` during transfer, and return
  to `READY` afterward so the same buffer can be downloaded again.
- Preserve frozen timing metadata and keep output off the ThingSet interface.

Commit 3: `feat: stream scope captures on the data serial port`

Gate: exact header/count, successful PlatformIO filter conversion to 1,024 by
8, and responsive ThingSet during transfer.

### Phase 4 — Add Python support

- Add the explicit-port serial parser with injectable fake transport.
- Extend `PowerTestBench` and validate non-boolean integer decimation 1-100.
- Add duration-aware polling, metadata/payload verification, malformed-input
  tests, and safe cleanup.
- Extend the example with `--scope-port`, `--capture`, `--pretrigger`,
  `--decimation`, and optional CSV output. Capture does not imply power-on.

Commit 4: `feat: add Python decimated scope capture interface`

Gate: Python compilation and all `unittest` cases pass.

### Phase 5 — Add MATLAB support

- Add the equivalent MATLAB serial class, parser, capture struct, fake
  transport, PowerTestBench methods, validations, and cleanup.
- Extend the callable example with equivalent name-value options.

Commit 5: `feat: add MATLAB decimated scope capture interface`

Gate: `checkcode` introduces no issues and all `matlab.unittest` cases pass.

### Phase 6 — Document and accept

- Update both READMEs with the dual-CDC architecture, object tree, decimation
  table, protocol, APIs, examples, memory cost, explicit-port rule, and
  1200-baud warning.
- Change the old4 scope rows to present and remove scope from the parity
  backlog only after acceptance.
- Complete this ledger with final evidence.

Commit 6: `docs: document and validate decimated scope capture`

Gate: all Python and MATLAB tests, `pio run -e USB`, hardware acceptance, and
worktree-isolation checks pass.

## Hardware and Test Acceptance

Use only these identity-pinned devices:

- TWIST target, USB serial `5843500300470047`:
  - data/scope:
    `/dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_5843500300470047-if00`
  - ThingSet:
    `/dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_5843500300470047-if02`
- Protected OWNVERTER, USB serial `584350030047002D`: never open, probe, reset,
  or flash.

Before every flash or reconnection, resolve the stable path and verify product,
serial, VID `2fe3`, and PID `0100`. Never fall back to `ttyACM*`, VID/PID-only
selection, or wrapper auto-detection. Temporarily select TWIST with
`board_id = 5843500300470047`, verify the upload log, and restore
`platformio.ini`. Flashing requires no user confirmation, but the test never
controls the external supply.

Required coverage includes:

- State transitions, invalid ordering, invalid decimation, active-write
  rejection, re-arm, repeated download, and shell responsiveness.
- Python and MATLAB decimation validation for booleans, non-integers, 0, 1,
  10, 100, 101, NaN, and infinity.
- Valid, wrapped, truncated, malformed, non-hex, wrong-count, wrong-channel,
  timeout, and probe-mismatch parsing.
- Missing-scope-client errors and cleanup after exceptions.
- Pre-trigger-zero timing:
  - decimation 1: 100 us period, 102.4 ms nominal duration, ready in
    0.08-0.25 s;
  - decimation 10: 1000 us, 1.024 s, ready in 0.9-1.2 s;
  - decimation 100: 10000 us, 10.24 s, ready in 9.8-10.7 s.
- Trigger requests between acquisition ticks latch until the next sample.
- At the user-provided 30 V input with no load, keep `POWER_OFF` and both legs
  disabled; require finite samples, mean VHigh within 5% of 30 V, and current
  magnitudes below 0.5 A.
- Download the same buffer with Python and MATLAB and require identical
  channels, dimensions, final index, active decimation, sample period, and
  decoded samples.
- Perform no frequency write and do not make the known frequency issue part of
  scope acceptance.
- Verify OWNVERTER remains unchanged after the test.

## Scope Assumptions

- Maximum capture duration is exactly 10.24 s at decimation 100.
- Decimation changes scope sampling frequency, not critical-task frequency or
  buffer size.
- The protocol uses legacy text/hex, ThingSet acquisition control, both host
  languages, a fixed 1,024 by 8 capture, and a permanent `?` probe.
- The exact tracked filename is uppercase
  `THINGSET_COMM_PROTOCOL_PLAN.md`.
- Scope plotting and reference-sweep automation remain out of scope; the
  existing PlatformIO filter may continue plotting legacy-format captures.
