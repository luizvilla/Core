# 30 V TWIST Buck-Regulation Hardware Acceptance Test

## Overall status

- Status: **PASSED** (frequency write/readback waived by user)
- Started: `2026-07-29T15:19:15+02:00`
- Finished: `2026-07-29T15:46:21+02:00`
- Operator-controlled input supply: energized at nominal `30 V` on the high side
- Load condition: no external load

## Run metadata

| Field | Value |
|---|---|
| Git SHA | `ad6d44dc759df191fd4ba9b6646d5dc89f6802ed` |
| Branch | `serial_thingset_example` |
| Required baseline | `ad6d44d` (verified ancestor of `HEAD`) |
| Controller board | `spin` `1_2_0` |
| Shield | `twist` `1_4_2` |
| Firmware environment | `USB` |
| Python | `3.12.3` |
| MATLAB | `R2024b Update 2` (`24.2.0.2773142`) |
| PlatformIO | Core `6.1.19` |

## Device selection and isolation

| Role | VID:PID | Product | USB serial | Stable path | Initial target | Policy |
|---|---|---|---|---|---|---|
| Target | `2fe3:0100` | `TWIST_V1_4_2` | `5843500300470047` | `/dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_5843500300470047-if02` | `/dev/ttyACM1` | Explicit connections only |
| Protected | `2fe3:0100` | `OWNVERTER_V1_1_0` | `584350030047002D` | `/dev/serial/by-id/usb-OwnTech_Technologies_OWNVERTER_V1_1_0_584350030047002D-if00` | `/dev/ttyACM2` | **DO_NOT_TOUCH** |
| Protected (appeared during run) | `2fe3:0100` | `TWIST_V1_4_2` | `423250070031003C` | `/dev/serial/by-id/usb-OwnTech_Technologies_TWIST_V1_4_2_423250070031003C-if00` | `/dev/ttyACM3` | **DO_NOT_TOUCH** |

Initial read-only enumeration at `2026-07-29T15:19:15+02:00`:

```text
/dev/ttyACM0  TWIST_V1_4_2      VID:PID=2FE3:0100  SER=5843500300470047  LOCATION=3-4:1.0
/dev/ttyACM1  TWIST_V1_4_2      VID:PID=2FE3:0100  SER=5843500300470047  LOCATION=3-4:1.2
/dev/ttyACM2  OWNVERTER_V1_1_0  VID:PID=2FE3:0100  SER=584350030047002D  LOCATION=3-6.3:1.0
```

OWNVERTER initial snapshot:

- Product: `OWNVERTER_V1_1_0`
- USB serial: `584350030047002D`
- Stable path: `/dev/serial/by-id/usb-OwnTech_Technologies_OWNVERTER_V1_1_0_584350030047002D-if00`
- Resolved target: `/dev/ttyACM2`
- Opened or probed by this test: **No**

Additional device observation:

- TWIST serial `423250070031003C` first appeared at `2026-07-29T15:33+02:00`.
- It is not the target and is protected from all serial opens, resets, and uploads.

## Safety notes

- The physical 30 V supply is controlled only by the user; this test does not switch it.
- Live conversion tests run one leg at a time with no external load.
- Wrapper auto-detection and direct `/dev/ttyACM*` connections are forbidden.
- `PowerAPI::disconnectDriver()` currently calls `setPin()` instead of clearing the pin. The test therefore uses `POWER_OFF`, normal wrapper shutdown without hardware disconnect, and an MCU reset to establish the final driver state.
- Calibration values are read-only during this test.
- Per user direction at `2026-07-29`, frequency write/readback testing is waived because of a known firmware issue. The live tests use and retain the compiled `200000 Hz` default. This waiver does not hide the observed attempt-1 failure.
- On a live-test exception: request normal shutdown, clear only the selected capacitor, close the target port, reset TWIST, and revalidate its identity before reconnecting.

## Original converter metadata

Captured before the first metadata write and verified after final reset.

| Field | Original value | Temporary value | Restored |
|---|---|---|---|
| Board name | `TWIST` | `TEST-TWIST` | **Yes — post-reset readback** |
| Board version | `v1.4.2` | `TEST-1` | **Yes — post-reset readback** |
| Serial number | `UNSET` | `WRAPPER-TEST` | **Yes — post-reset readback** |
| Firmware version | `1.0.0` | `TEST-1` | **Yes — post-reset readback** |

## Phase progress

| # | Phase | Status | Evidence |
|---:|---|---|---|
| 1 | Device identity and isolation | **PASS** | Exact products and USB serials distinguished; stable paths recorded; no serial port opened |
| 2 | Preflight | **PASS** | Clean initial worktree; baseline verified; 12 Python and 9 MATLAB tests passed |
| 3 | Build | **PASS** | USB firmware build succeeded in 78.24 s; signed image created |
| 4 | Flash | **PASS** | Upload script explicitly selected TWIST serial `5843500300470047`; upload and reset succeeded |
| 5 | Post-flash identity | **PASS** | Exact TWIST shell path returned after image-swap reboot; OWNVERTER unchanged |
| 6 | Python configuration | **PASS — FREQUENCY WAIVED** | Explicit TWIST connection; discovery, safe state, calibration reads, 30 V gate, staged leg configuration, and temporary metadata write passed at 200 kHz |
| 7 | Python Leg 1 regulation | **PASS** | Settled in 0.304 s; 20-sample mean 4.978 V; every criterion passed; cleanup succeeded |
| 8 | Metadata persistence | **PASS** | MATLAB read all four Python temporary values after confirmed MCU reset |
| 9 | MATLAB configuration | **PASS** | Explicit port, reset-state verification, calibration reads, staged leg settings, and 30 V gate passed at 200 kHz |
| 10 | MATLAB Leg 2 regulation | **PASS** | Settled in 0.366 s; 20-sample mean 4.975 V; every criterion passed; cleanup succeeded |
| 11 | Metadata restoration | **PASS** | MATLAB restored all four originals; Python and MATLAB verified them after confirmed MCU reset |
| 12 | Static measurements | **PASS** | 20 Python plus 20 MATLAB complete samples; all finite; every high-side sample within 28.5–31.5 V; high-side means differed by 0.042 V |
| 13 | Final safe state | **PASS** | `POWER_OFF`, 200 kHz, both enables/drivers/capacitors false, original metadata restored |
| 14 | OWNVERTER final verification | **PASS** | Product, serial, location, and stable path unchanged; no command opened or targeted it |

## Command results

| Timestamp | Command | Result | Notes |
|---|---|---|---|
| `2026-07-29T15:19:15+02:00` | `git status --short --branch` | PASS | Initial tracked worktree clean |
| `2026-07-29T15:19:15+02:00` | `git merge-base --is-ancestor ad6d44d HEAD` | PASS | Exit status 0 |
| `2026-07-29T15:19:15+02:00` | `python3 -m serial.tools.list_ports -v` | PASS | Exact TWIST and OWNVERTER identities found |
| `2026-07-29T15:20:00+02:00` | MATLAB `runtests('src/tools/tests')` | PASS | 9 passed, 0 failed, 0 incomplete |
| `2026-07-29T15:20:15+02:00` | Python `unittest discover` | PASS | 12 tests passed |
| `2026-07-29T15:20:21+02:00` | `pio run -e USB` | PASS | 78.24 s; flash 80.5%, RAM 41.4% |
| `2026-07-29T15:22:15+02:00` | `git diff -- platformio.ini` | PASS | Exactly one line set `board_id = 5843500300470047` |
| `2026-07-29T15:22:15+02:00` | Pre-upload USB enumeration | PASS | Both exact identities unchanged |
| `2026-07-29T15:22:15+02:00` | `pio run -e USB -t upload` | PASS | Upload completed in 47.12 s; target selection evidence below |
| `2026-07-29T15:23:02+02:00` | Immediate post-reset enumeration | EXPECTED TRANSIENT | TWIST was temporarily absent during first image-swap boot; OWNVERTER remained present |
| `2026-07-29T15:23:12+02:00` | Post-flash identity gate | PASS | TWIST exact `if02` path restored; OWNVERTER unchanged |
| `2026-07-29T15:24:18+02:00` | Python configuration attempt 1 | FAIL | Readback error: `Config/Frequency_Hz` was restored to `200000`, requested `100000`; no PWM started and metadata write had not run |
| `2026-07-29T15:26:05+02:00` | Python configuration attempt 2 | FAIL | Multi-field Leg 1 update returned `0xA0 Bad Request`; no PWM or hardware connection started |
| `2026-07-29T15:28:19+02:00` | Baseline firmware restore build/upload | PASS | Temporary frequency workaround removed; TWIST serial explicitly selected; firmware returned to baseline behavior and 200 kHz default |
| `2026-07-29T15:29:42+02:00` | Python compile/unit tests after serial staging fix | PASS | Compilation passed; 12 tests passed |
| `2026-07-29T15:29:42+02:00` | MATLAB `checkcode`/unit tests after serial staging fix | PASS | Static analysis clean; 9 passed, 0 failed |
| `2026-07-29T15:30:25+02:00` | Python configuration attempt 3 | PASS | Multi-field wrapper calls succeeded through staged transport writes; metadata readback matched |
| `2026-07-29T15:42+02:00` | Python post-reset metadata and static measurement test | PASS | Originals persisted; 20 complete samples; final state safe |
| `2026-07-29T15:43+02:00` | MATLAB static measurement attempt 1 | FAIL | Harness asserted nonexistent `wCapacitor`; firmware readback field is `wCapa`; stopped before sample collection |
| `2026-07-29T15:44+02:00` | MATLAB static measurement attempt 2 | PASS | 20 complete samples; metadata and final safe state verified |
| `2026-07-29T15:45+02:00` | Final Python compilation/unit tests | PASS | Compilation passed; 12 tests passed |
| `2026-07-29T15:45+02:00` | Final MATLAB `checkcode`/unit tests | PASS | Static analysis clean; 9 passed, 0 failed |
| `2026-07-29T15:45+02:00` | Host `/usr/bin/pio run -e USB` | FAIL | Obsolete PlatformIO 4.3.4 launcher was incompatible with installed Click; no build or hardware operation started |
| `2026-07-29T15:46:21+02:00` | PlatformIO 6.1.19 `run -e USB` | PASS | USB build succeeded in 30.15 s; flash 80.5%, RAM 41.4% |
| `2026-07-29T15:46:21+02:00` | Final read-only USB enumeration | PASS | Target and OWNVERTER exact identities/stable paths present; upload selector restored |

Build artifacts:

- `firmware.bin`: SHA-256 `74e9bc740f76a5e48a9f1b3830412179f788101a539df0953f0c507fe7eaef35`
- Preflight `firmware.mcuboot.bin`: SHA-256 `f73ba5869508c027be6a7d8543927e12238aaccee66fb7f6d948b72f0bfca296`
- Final `firmware.mcuboot.bin`: SHA-256 `f169ae3e75fc1901bd0de04b4d91671c9848eb2a70eea8b8b74c714a5f5c9cc8`

## Upload-selection evidence

```text
Preferred board for upload: 5843500300470047
Detecting Spin boards connected to host...
  Board 1: Found Spin board on port /dev/ttyACM2 with unique ID 584350030047002D
  Board 2: Found Spin board on port /dev/ttyACM1 with unique ID 5843500300470047
  Board 3: Found Spin board on port /dev/ttyACM0 with unique ID 5843500300470047
Board with unique ID 5843500300470047 was found and selected for upload
Forcing reset using 1200bps open/close on port /dev/ttyACM0
...
USB SUCCESS
```

The selected upload interface belonged to the required TWIST USB serial.

## Python Leg 1 regulation

| Metric | Result | Limit |
|---|---:|---:|
| Five-sample settling time | `0.304 s` | `<= 1.0 s` |
| V1 minimum | `4.64 V` | `>= 4.5 V` |
| V1 mean | `4.9775 V` | `4.75–5.25 V` |
| V1 maximum | `5.22 V` | `<= 5.5 V` |
| V1 peak-to-peak | `0.58 V` | `<= 1.0 V` |
| `rV1Max_V` before shutdown | `5.39 V` | `4.5–6.5 V` |
| Duty minimum/mean/maximum | `0.148 / 0.149 / 0.150` | mean `0.05–0.30` |
| Maximum absolute `rI1Low_A` | `0.14 A` | `< 0.5 A` |
| Maximum absolute `rI2Low_A` | `0.31 A` | `< 0.5 A` |
| Maximum absolute `rIHigh_A` | `0.18 A` | `< 0.5 A` |
| Unselected Leg 2 | disabled throughout | required |
| Cleanup | `POWER_OFF`, both legs disabled, Leg 1 capacitor disconnected | required |

Pre-test `rVHigh_V` was `29.92 V`. All 20 complete samples were finite. The full 20-sample V1 range was `4.64–5.22 V`. After cleanup, `wDriver` remained true as expected from the known disconnect limitation; an identity-pinned TWIST console reset was issued, followed by successful TWIST and OWNVERTER identity revalidation.

## MATLAB Leg 2 regulation

| Metric | Result | Limit |
|---|---:|---:|
| Five-sample settling time | `0.366 s` | `<= 1.0 s` |
| V2 minimum | `4.75 V` | `>= 4.5 V` |
| V2 mean | `4.9750 V` | `4.75–5.25 V` |
| V2 maximum | `5.25 V` | `<= 5.5 V` |
| V2 peak-to-peak | `0.50 V` | `<= 1.0 V` |
| `rV2Max_V` before shutdown | `5.47 V` | `4.5–6.5 V` |
| Duty minimum/mean/maximum | `0.169 / 0.16915 / 0.170` | mean `0.05–0.30` |
| Maximum absolute `rI1Low_A` | `0.12 A` | `< 0.5 A` |
| Maximum absolute `rI2Low_A` | `0.34 A` | `< 0.5 A` |
| Maximum absolute `rIHigh_A` | `0.20 A` | `< 0.5 A` |
| Unselected Leg 1 | disabled throughout | required |
| Cleanup | `POWER_OFF`, both legs disabled, Leg 2 capacitor disconnected | required |

Pre-test `rVHigh_V` was `29.92 V`. All 20 complete samples were finite. The full 20-sample V2 range was `4.75–5.25 V`. Verbose transport evidence showed complete valid JSON for every accepted measurement.

## Static measurements

Each wrapper collected 20 complete measurement sets at 100 ms intervals after a
confirmed MCU reset. Both explicitly used the target TWIST `by-id` shell path.

| Measurement | Python min / mean / max | MATLAB min / mean / max |
|---|---:|---:|
| `rVHigh_V` | `29.68 / 29.945 / 30.18 V` | `29.68 / 29.987 / 30.18 V` |
| `rV1Low_V` | `0.30 / 0.6145 / 0.83 V` | `0.52 / 0.675 / 0.88 V` |
| `rV2Low_V` | `-0.26 / -0.0035 / 0.19 V` | `-0.17 / -0.0535 / 0.15 V` |
| `rI1Low_A` | `-0.11 / -0.0305 / 0.05 A` | `-0.06 / 0.015 / 0.07 A` |
| `rI2Low_A` | `0.14 / 0.2105 / 0.30 A` | `0.13 / 0.205 / 0.27 A` |
| `rIHigh_A` | `-0.02 / 0.098 / 0.20 A` | `-0.04 / 0.074 / 0.16 A` |
| `rTemp1_degC` | `36.12 / 36.22 / 36.37 °C` | `36.14 / 36.145 / 36.16 °C` |
| `rTemp2_degC` | `38.36 / 38.408 / 38.44 °C` | `38.48 / 38.589 / 38.70 °C` |
| `rV1Max_V`, `rV2Max_V`, duties | `0 / 0 / 0` | `0 / 0 / 0` |

- All 40 high-side voltage samples were within the required `28.5–31.5 V`.
- Python/MATLAB mean `rVHigh_V` difference: `0.042 V` (limit `<= 1 V`).
- Every returned measurement was finite.
- The nonzero unloaded low-side/current offsets above are real ADC readings, not
  synthesized values.
- Both passes ended in `POWER_OFF` with both leg enables, drivers, and capacitors
  false.

## Final device isolation snapshot

At `2026-07-29T15:46:21+02:00`:

```text
/dev/ttyACM0  TWIST_V1_4_2      SER=5843500300470047  LOCATION=3-4:1.0
/dev/ttyACM1  TWIST_V1_4_2      SER=5843500300470047  LOCATION=3-4:1.2
/dev/ttyACM2  OWNVERTER_V1_1_0  SER=584350030047002D  LOCATION=3-6.3:1.0
/dev/ttyACM4  MCUBOOT            SER=423250070031003C  LOCATION=3-5.3.1:1.0
```

The target shell stable path still resolves to `/dev/ttyACM1`. The protected
OWNVERTER stable path still resolves to `/dev/ttyACM2`, with its original
product and serial. The additional protected TWIST was observed in its MCUBOOT
state at the final snapshot; no command from this test opened, reset, flashed,
or otherwise targeted serial `423250070031003C`.

## Incidents and recovery actions

Append-only log.

1. `2026-07-29T15:24:18+02:00` — Python configuration attempt 1 failed on frequency readback. The configuration callback queried an HRTIM maximum-frequency field that remains zero after initialization, so every positive changed frequency was treated as above maximum and restored. The wrapper correctly surfaced the rejection. Safe shutdown had already completed, both legs remained disabled, metadata was unchanged, and no PWM was started. Recovery: apply a minimal callback guard that enforces the upper limit only when the lower-level API reports a nonzero maximum, rebuild, reflash the explicitly selected TWIST, and repeat the phase. This incident and its original failure remain part of the record.

2. `2026-07-29` — The user identified frequency as a known issue and directed that its test be ignored for now. The temporary callback workaround was removed before continuing, and the acceptance scope was changed to retain the compiled `200000 Hz` default without exercising frequency writes. The original frequency failure remains recorded but is waived from overall pass/fail.

3. `2026-07-29T15:26:05+02:00` — A multi-field Python leg update failed with `0xA0 Bad Request`; focused diagnostics showed individual field updates passed while burst updates timed out or corrupted the shell command. The firmware has a 64-byte serial receive ring and a 256-byte ThingSet command buffer. Recovery: both generic clients now preserve `write(path, values)` while staging multi-field updates as individual requests and pacing serial data in 32-byte chunks. Python compilation/12 tests and MATLAB `checkcode`/9 tests passed, and the same real multi-field Python configuration then succeeded.

4. `2026-07-29T15:32:55+02:00` — MATLAB correctly blocked before metadata verification or Leg 2 power because the first attempted console reset had not reset the target; Leg 1 `wDriver` was still true. MATLAB cleanup completed with `POWER_OFF`, both legs disabled, and no Leg 2 PWM start. Recovery: use an identity-pinned baseline reflash as the reliable MCU reset, then verify both driver flags are false before retrying MATLAB.

5. `2026-07-29T15:33+02:00` — A third OwnTech USB device appeared during the run: TWIST serial `423250070031003C`, stable `if00` path resolving to `/dev/ttyACM3`. It was added to the protected device table immediately. No serial command has opened or targeted it.

6. `2026-07-29T15:35:43+02:00` — MATLAB retry verified reset state, verified all four Python temporary metadata values after the reset, restored all original metadata, configured both legs, and settled Leg 2 into range in `0.333 s`. The temporary test harness then raised `MATLAB:heterogeneousStrucAssignment` while building its 20-sample struct array. Wrapper cleanup completed: `POWER_OFF`, both legs disabled, Leg 2 capacitor disconnected; no acceptance window was claimed. Recovery: store the varying decoded sample structs in a cell array, reset the explicitly selected target again to clear the driver state, and repeat the MATLAB test.

7. `2026-07-29T15:38:05+02:00` — The next MATLAB retry confirmed restored metadata persisted across reset and settled Leg 2 in `0.312 s`, but one acceptance-window `readMeasurements()` call returned a non-struct and `PowerTestBench` raised its intended readback error. Cleanup again completed with `POWER_OFF`, both legs disabled, and Leg 2 capacitor disconnected. A subsequent 100-read MATLAB `POWER_OFF` diagnostic produced `0` malformed reads. Recovery: capture verbose transport evidence during a reset-clean live retry before deciding whether the transient requires a client change.

8. `2026-07-29T15:38:45+02:00` — A reset-only baseline upload selected target serial `5843500300470047` but stalled at 33% and was manually interrupted. A non-target TWIST symlink was transiently absent at the preceding read-only gate; because the gate and upload were separated instead of chained, the upload still began. The uploader nevertheless explicitly selected only the target serial, and no protected serial was opened. TWIST safely booted its previously confirmed active image with both drivers, capacitors, and enables false; original metadata and 200 kHz were verified. Recovery and final reset used a 1200-baud touch on the exact target `if00`, verified the resulting `MCUBOOT` product with serial `5843500300470047`, and issued `mcumgr reset` through that stable MCUBOOT `by-id` path without transferring firmware.

9. `2026-07-29T15:43+02:00` — The first MATLAB static harness attempt checked a nonexistent `wCapacitor` readback field; the firmware object is named `wCapa`. The assertion failed before the 20-sample loop. The preceding state read proved `POWER_OFF`, both drivers false, and both enables false; wrapper cleanup repeated normal shutdown. Recovery: correct the harness-only field name and rerun. The corrected 20-sample pass succeeded.

10. `2026-07-29T15:45+02:00` — The first final build command resolved `/usr/bin/pio`, an obsolete PlatformIO 4.3.4 launcher incompatible with the installed Click version. It failed during Python import, before any project build or hardware access. Recovery: explicitly use the established PlatformIO 6.1.19 executable. The final USB build then passed in 30.15 seconds.

## Final checklist

- [x] Every upload and wrapper connection targeted TWIST serial `5843500300470047`.
- [x] No operation targeted OWNVERTER serial `584350030047002D`.
- [x] No operation targeted non-target TWIST serial `423250070031003C`.
- [x] OWNVERTER identity and stable path remained unchanged.
- [x] Python unit tests passed (`12` expected).
- [x] MATLAB unit tests passed (`9` expected).
- [x] Firmware build passed.
- [x] Firmware upload passed with explicit TWIST selection.
- [x] Python metadata write and MATLAB persistence readback passed.
- [x] Original metadata was restored and verified after reset.
- [x] Python Leg 1 regulated `V1` to `5 V` within all limits.
- [x] MATLAB Leg 2 regulated `V2` to `5 V` within all limits.
- [x] Static 30 V measurements met all limits.
- [x] Frequency remained at the compiled `200000 Hz` default (write/readback test waived).
- [x] TWIST ended in `POWER_OFF` with both legs disabled.
- [x] Original `platformio.ini` was restored.
- [x] No required evidence remains pending.

---

# Dual-Serial Scope Acceptance Run

## Scope-run status

- Status: **BLOCKED — OFFLINE IMPLEMENTATION COMPLETE**
- Recorded through: `2026-07-29T17:52:43+02:00`
- Baseline implementation SHA: `88d7ba0a7fcbf3b0c1bfeb331678233c401d5133`
- Branch: `serial_thingset_example`
- External supply: user-controlled; this run issued no supply command
- Frequency: no write attempted
- Converter state during scope acceptance: intended `POWER_OFF`, both legs
  disabled

This section is append-only and independent of the completed buck-regulation
run above. The scope firmware and both host interfaces are implemented and
pass their offline gates. Live scope acceptance is blocked because the first
Phase 2 image reserved too much Zephyr heap and the target application wedged
before its serial tasks started. Its normal 1200-baud bootloader callback
cannot currently be reached; a physical reset of the target TWIST is required
before reflashing the corrected image.

## Scope device isolation

| Role | Product | USB serial | Stable interface | Status |
|---|---|---|---|---|
| Scope target | `TWIST_V1_4_2` | `5843500300470047` | `...5843500300470047-if00` data, `...5843500300470047-if02` ThingSet | Enumerated; application endpoints wedged |
| Protected | `OWNVERTER_V1_1_0` | `584350030047002D` | `...584350030047002D-if00` | **DO_NOT_TOUCH**, normal and unchanged |
| Protected | `TWIST_V1_4_2` | `423250070031003C` | `...423250070031003C-if00` | **DO_NOT_TOUCH**, normal and unchanged |

Read-only snapshot at `2026-07-29T17:52:43+02:00`:

```text
/dev/ttyACM0  TWIST_V1_4_2      SER=5843500300470047  LOCATION=3-4:1.0
/dev/ttyACM1  OWNVERTER_V1_1_0  SER=584350030047002D  LOCATION=3-6.3:1.0
/dev/ttyACM2  TWIST_V1_4_2      SER=5843500300470047  LOCATION=3-4:1.2
/dev/ttyACM3  TWIST_V1_4_2      SER=423250070031003C  LOCATION=3-5.3.1:1.0
```

No scope-run command opened, probed, reset, or flashed serial
`584350030047002D` or serial `423250070031003C`.

## Scope progress ledger

| Phase | Status | Evidence | Next action |
|---:|---|---|---|
| 0. Plan and convention | PASS | Plan-only commit `a6ca0f7`; global commit-sequence rule stored outside the repository | Complete |
| 1. Data-port viability | PASS | Target-only upload selected `5843500300470047`; three exact `SCOPE-DATA/1 OK` probes, unknown-command response, concurrent `/Converter` read on `if02`, and zero cross-port bytes | Complete |
| 1A. Upload isolation remediation | PASS | Commits `a47a096` and `88d7ba0`; explicit target fallback removed, uploader waits for matching serial plus `MCUBOOT`, and Linux `mcumgr` uses the serial-specific by-id path | Use remediated uploader after physical target reset |
| 2. Decimated acquisition | BLOCKED | Corrected image builds at 41.7% reported RAM with 4 KiB system heap; live state/timing gate cannot run on wedged target | Physical target reset, flash, then test decimations 1/10/100 |
| 3. Bounded download | BLOCKED | Firmware emits exact header, 32,768 buffer bytes as 8,192 hex lines, and repeatable `READY` ownership; build passes | Run live transfer, repeated-download, and shell-responsiveness gates |
| 4. Python wrapper | PASS | `compileall` and 23 `unittest` cases pass | Run one live download |
| 5. MATLAB wrapper | PASS | Zero `checkcode` findings and 19 `matlab.unittest` cases pass | Download the same frozen live capture |
| 6. Documentation/final acceptance | BLOCKED | READMEs, protocol, timing, APIs, examples, and this evidence record updated; 23 Python tests, 19 MATLAB tests with zero analyzer findings, and USB build at 41.7% RAM/81.6% flash pass | Complete live gates and mark scope rows present |

## Scope incidents and recovery

1. The first Phase 2 compile used unavailable minimal-C++ headers. It was
   corrected to C `math.h`/`stdint.h` interfaces.
2. The first Phase 2 image used a 40 KiB Zephyr system heap. ScopeMimicry
   allocates through newlib `malloc`, leaving insufficient newlib heap for its
   32,768-byte buffer plus overhead. The corrected configuration retains the
   existing 4 KiB system heap and leaves otherwise-unused RAM to newlib.
3. During that upload, the protected OWNVERTER temporarily enumerated as
   `MCUBOOT`. Inspection showed the uploader accepted the still-enumerated
   application `/dev/ttyACM*` and retained that unstable path. No explicit
   command targeted the protected serial. The uploader was fixed to wait for
   the selected serial and `MCUBOOT` product; OWNVERTER returned to its normal
   identity without being opened or reset by the test.
4. The target application remained wedged. Two target-only 1200-baud attempts
   could not reach its bootloader callback. No further flash was attempted.
5. The first MATLAB fake-parser run used double-quoted `sprintf` strings where
   byte conversion required characters. The fixtures were corrected; all 19
   tests passed on rerun.

## Remaining live acceptance checklist

- [ ] Physically reset TWIST serial `5843500300470047`.
- [ ] Revalidate all three USB identities and stable paths.
- [ ] Flash the corrected image using the identity-pinned uploader.
- [ ] Verify the complete `/Debug/Scope` tree and invalid-state restoration.
- [ ] Verify 100, 1,000, and 10,000 us sample-period readbacks.
- [ ] Verify ready windows of 0.08–0.25 s, 0.9–1.2 s, and 9.8–10.7 s.
- [ ] Verify a trigger between decimated ticks is retained.
- [ ] Verify exact 8,192-value and repeated downloads with responsive `if02`.
- [ ] Decode the same frozen buffer with Python and MATLAB and compare it
  exactly.
- [ ] At user-provided 30 V/no-load input, verify finite samples, mean VHigh
  within ±5%, and current magnitudes below 0.5 A while power remains off.
- [ ] Re-enumerate and confirm both protected devices remain unchanged.
