# Extend Python and MATLAB ThingSet Clients

## Summary

Add persistent writable converter metadata, matched Python/MATLAB
power-test-bench wrappers, safe examples, tests, and documentation. Preserve
the generic ThingSet APIs and require explicit opt-in before energizing
hardware.

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
- No reference-sweep or plotting client is included.
- The exact tracked filename is `THINGSET_COMM_PROTOCOL_PLAN.md`.
