# SPOC safety bench (L0)

The SPOC bench runs the Safety API's L0 tests ([ADR 0009](../adr/0009-safety-api-hardware-protection.md)) on a bare Spin with a scope. This is a living document: the plan, the decisions, then the results as each step lands.

Issues: luizvilla/Core#26 (bench), luizvilla/Core#23 (generic firmware and L0 runner) · tracking: luizvilla/Core#14

## Bench

| Item | Identification | Role |
|---|---|---|
| CUT Spin | USB 2fe3:01ff | Card under test: runs the test firmware |
| MUX Spin | USB 2fe3:0100 (Core-MUX from the branch; 0x0102 before the reflash of 2026-11-19) | Routes CUT pins to the four scope channels; also the independent injector (TP5) |
| Rigol MHO984 | USB 1ab1:0452, `/dev/usbtmc0` | Measures; verdicts come from its own measurements |

Ports are found by USB product ID, never by a fixed `ttyACM` number. Wiring: ADR 0009, implementation note of 2026-11-17.

### Scope link (MHO984 over Linux usbtmc)

Found on the Core4 `test/twt-bnc` bench (`tests/twt_bnc/README.md` there, fw 00.01.00) and seen again here on 2026-11-18:

- The first reply after plugging the scope in (or after a USBTMC clear) is lost. The driver sends `*IDN?` until one comes back and never sends a clear.
- Once the kernel driver is out of step with the scope, every query times out until the USB cable is replugged.
- A screenshot (~130 kB) puts the driver out of step: screenshots are off by default.
- The scope never answers `:SYSTem:ERRor?` or `*ESR?`: setups are checked by reading them back.
- A query the scope does not know (seen with `:SYSTem:BANDwidth?`) gets no answer and leaves the link out of step: the next connection fails until a replug or a few minutes' wait. Send only queries from the MHO900 programming guide.
- pyvisa-py does not list the scope while the kernel `usbtmc` driver holds it: the driver talks to `/dev/usbtmc0` directly.

## Decisions

- **2026-11-18** (Luiz Villa):
  - the scope driver is ported from Core4's `rigol_scope.py` (usbtmc), not SPOC_Test_Software's pyvisa client;
  - the `Core-MUX` injector is written on a branch of `owntech-foundation/SPOC_Software` off `Core-MUX`, its hash pinned in Core2;
  - the existing L0 firmware (`safety_comparator`, `safety_supervision`) stays as it is; the generic firmware is new.

## Plan and commits

### Phase 0: bring-up (#26)

| # | Commit | Content |
|---|---|---|
| C1 | `docs(safety): SPOC bench plan and commit sequence` | This document; ADR 0009 wiring note; MHO984 named |
| C2 | `test(scope): MHO984 driver over usbtmc` | `tests/common/rigol_mho.py`, ported from Core4, with a capture command line and the udev rule |
| C3 | `test(spoc): SPOC host client and preflight` | `tests/common/spoc.py`: ports by product ID, MUX routing over ThingSet; `tests/common/spoc_preflight.sh` |
| C4 | `docs(safety): SPOC chain smoke test, scope vs 31 ns verdict` | Stock Core-CUT and Core-MUX with one PWM test; MHO984 bandwidth and sample rate against the 31 ns comparator verdict |

### Phase 1: injection and calibration (#26)

| # | Commit | Content |
|---|---|---|
| C5 | `feat(mux): injector, OPAMP4 follower and GPIO step` (SPOC_Software branch) | DAC4_CH1 → OPAMP4 → PB12; GPIO step on PB13; one source at a time, the idle one in analog mode, source changed only with CH3's mux disabled |
| C6 | `test(spoc): pin Core-MUX injector, injector client` | Hash pinned; injector calls in `spoc.py` |
| — | Wiring (by hand) | TP1–TP3; PB12 through 220–470 Ω to TP5; PB13 through ~100 Ω to TP5; TP7 for CH4 |
| C7 | `test(safety): L0-CAL, trim and skew/gain calibration` | Trim the four channels (Core4 trimming flow), then one CUT pin on all four: skew and gain stored with the campaign |
| C8 | `docs(safety): SPOC bench README, photo, ADR note` | L0 runner README with the photo; closes #26 |

### Phase 2: generic firmware and L0 runner (#23)

| # | Commit | Content |
|---|---|---|
| C9 | `feat(test): generic safety test firmware, ThingSet control` | `tests/firmware/safety_l0`: arm, PWM, report and clear faults, any shield |
| C10 | `feat(test): software fault injection in the generic firmware` | Stack overflow, hard fault, interrupts masked, watchdog starved, wild register writes after locking |
| C11 | `test(safety): L0 runner, cases from the manifest` | Limit × sign × path, self-injection through TP1–TP3, routes set with the PWM stopped |
| C12 | `test(safety): CPU-dead cases from the MUX injector` | Injection on TP5 while the CUT is dead; unblocks L0-SUP-08.c |
| C13 | `test(safety): comparator checks on the scope` | Trip latency from the GPIO step; low-bound comparator in service |
| C14 | `test(safety): edge cases` | Below threshold; glitch shorter and longer than the filter; fault while arming, during clear, during dead time; two faults at once |
| C15 | `docs(safety): L0 SPOC campaign results` | Campaign under `tests/out/`, control center checked, results here, in the ADR and on the issues; closes #23 |

Each test from C7 on is in `tests/catalogue.json` and is done only when it shows in `tests/out/control-center.html` with its newest result and rendered report. Campaign folders are git-ignored: the numbers are copied here.

## Results and findings

### 2026-11-18, bench seen from the host

- Both Spins and the scope enumerate (IDs above).
- The first probes (plain writes and reads on `/dev/usbtmc0`) got no answer. A few minutes later the ported driver (C2), which repeats `*IDN?` until a reply comes, got one without a replug: `RIGOL TECHNOLOGIES,MHO984,MHO9B280300263,00.01.00`.
- Both Spins already run the stock SPOC firmware: the MUX exposes `Mux`, the CUT `Cut`. The ThingSet shell is on USB interface 2 of each board (MUX `/dev/ttyACM1`, CUT `/dev/ttyACM3` here); interface 0 is the Zephyr console.
- MUX routing from Core2 (C3): PA8 → CH1 reads back `mux1`, input 8, enabled; `disable_all()` clears it.
- `tests/common/spoc_preflight.sh`: all OK.

### 2026-11-18, first smoke test of the chain (C4, not finished)

Stock Core-CUT drives PA8 at 1 kHz, duty 0.5; the MUX routes PA8 to CH1; CH1 is set ×10 (the SPOC 9 MΩ divider into 1 MΩ), 1 V/div, 200 µs/div, edge trigger 1.65 V. The scope's measurements:

| Item | Measured | Expected |
|---|---|---|
| Frequency | 8.10 kHz | 1 kHz |
| Duty | no valid value | 50 % |
| VTOP / VBASE | 2.95 V / 2.26 V | ~3.3 V / 0 V |
| Rise / fall time | 3.2 µs / 38 µs | edges of a few ns behind an untrimmed divider |
| Sample rate at that timebase | 5 MSa/s | — |

The chain answers end to end, but CH1 does not show the PWM. To check next, in this order: CH1's BNC cable goes to SPOC BNC1; CH1 with the mux disabled, then with PA8 routed and the PWM off, then on; `CutPwmController.read_state()` after the command; the channel's divider trim (Core4 trimming flow). The diagnostic run that would have done this failed: the scope stopped answering after the unknown query above.

The scope's bandwidth and maximum sample rate against the 31 ns verdict are still to read (programming guide or the scope's About page).

### 2026-11-19, smoke test continued: nothing reaches the BNCs

Scope CH1–CH4 cabled to SPOC BNC1–BNC4 the evening before (Luiz Villa); CH2 and CH3 disconnected before the runs below. The scope answered again after the night, no replug. The CUT reads back `enabled=True, enabled_outputs=('PA8',)` after a 1 kHz, 0.5 command.

| Condition | Scope CH1 (×10) |
|---|---|
| all muxes disabled | 22 kHz, −0.0 to 1.5 V |
| PA8 routed to CH1, PWM off | 257 Hz, −0.1 to 1.4 V |
| PA8 routed to CH1, PWM on | 2.2 kHz, −0.2 to 1.5 V |

PA8 routed to ch1, ch2, then ch3 (the MUX reads back each route enabled): the scope's CH1–CH3 all show the same ~1 V of noise (~0.1 V at the BNC) in every case, and none shows 1 kHz. The cables are not the cause: unrouted channels are disabled, and a slow divider would round the edges, not change the frequency. The signal is lost before the BNCs: the CUT pin, the mux supply or enables, or the board seating. Next: a probe directly on the CUT's PA8, then on the mux side of the CH1 divider (TP1).

Spectrum of live screen waveforms (1000 points at 1 ms/div, scope running), PA8 at 1 kHz, 0.5:

| Condition | Scope channel | Level (×10) | 1 kHz bin / median bin | Strongest components |
|---|---|---|---|---|
| all muxes disabled | CH1 | 2.3–5.3 V, median 3.3 V | 11 / 5 | 100 Hz, 42 kHz |
| PA8 → ch1, PWM off | CH1 | 1.7–4.8 V | 3 / 5 | 100 Hz, 42 kHz |
| PA8 → ch1, PWM on | CH1 | 1.7–5.1 V | 7 / 5 | 100 Hz, 42 kHz |
| PA8 → ch4, PWM on | CH4 | −1.0–1.0 V | 8 / 3 | 100, 200, 300 Hz |

No 1 kHz on any route. The 100 Hz harmonics are mains pickup on a floating input: nothing drives the BNCs, and the path from the CUT pin to the BNC is open somewhere. Driver finding: `single()` followed by `waveform()` returned the same record four times in a row (identical samples); waveforms read while the scope runs do change. To fix in C2 before any capture-based verdict.

### 2026-11-19, probing the board: the MUX firmware was the fault

Hand probe on scope CH2 (×10, DC, 1 MΩ), ground on the SPOC ground. Spin pin numbers from `GpioHAL::getPinNumber()`: pin 12 = PA8; on the MUX Spin pin 24 = PC0 (mux1 enable of CH1, active low), pins 29, 30, 51, 52 = PA0–PA3 (CH1 select S0–S3).

| Step | Point | Reading | Meaning |
|---|---|---|---|
| 1 | CUT pin 12, PWM 1 kHz 0.5 | 1000 Hz, 50 %, 0.03–3.32 V | the CUT drives PA8 (a first reading of ±0.35 V was a loose ground clip) |
| 2 | TP2 | flat 0 V | TP2 is CH1's ground, as in ADR 0009 |
| 3 | TP1, PA8 routed to CH1 | flat 3.3 V on all 14 inputs of mux1, and with every mux disabled | the mux output does not follow the select |
| 4 | MUX pin 24, route on/off ×3 | 0 V in both states | the enable is not driven (setup should leave it at 3.3 V when disabled) |
| 5 | MUX pin 52, input 0/8 ×3 | 0 V in both states | the select is not driven |

The firmware first found on the MUX (USB 2fe3:0102) answered ThingSet and read routes back as set, but drove none of the probed pins. Reflashed with Core-MUX from `owntech-foundation/SPOC_Software` branch `Core-MUX` (`295a3f6`, Spin 1.2.0, built with PlatformIO 6.2 in a scratch worktree, uploaded by board ID 5843500300470031): the board now enumerates as 2fe3:0100.

After the reflash:

| Point | Reading |
|---|---|
| MUX pin 52, input 0/8 ×2 | 0 V / 3.39 V |
| BNC1 (scope CH1), PA8 routed, live waveform | 1 kHz square, 0.29 V / 3.48 V, 50 %; harmonics 3, 5, 7 kHz; overshoot −0.79 / +4.79 V (divider not trimmed) |

**The chain CUT → MUX → BNC → scope works.** Open points:
- **`:MEASure` is not reliable yet:** see the next section.
- **CH1 divider trim** (overshoot): C7.
- `single()` stale record (above): not reproduced after the reflash (three captures across route changes differ as they should).
- The old 0x0102 build is lost: no binary was kept.

### 2026-11-19, the scope's own measurements on the SPOC signal

Signal: PA8 at 1 kHz, 0.5, routed to CH1 (×10, DC, untrimmed divider), 1 ms/div, 2 MSa/s, 10 k points, edge trigger on CH1.

The screen waveform is right: 1000 points, 20 crossings of 1.65 V, every 500 µs; noise 0.2 V rms on each level. The scope's measurement engine is not:

| Setting | FREQuency (10 readings) | PDUTy | VTOP / VBASe |
|---|---|---|---|
| NORMal, running | 250–1000 Hz | 0.17–0.83 | 3.05–5.12 / −1.03–0.34 V |
| NORMal, BW limit 20 MHz | 333–1000 Hz | 0.50–0.75 | 3.05–3.93 / −0.70–0.35 V |
| HRESolution, BW off, running | 999.5–1000.5 Hz | 0.499–0.500 | 3.12–3.81 / −0.53–0.35 V |
| HRESolution, 3 × 7 readings | outliers 166.67 Hz; 0.083, 0.917 | | |
| HRESolution, stopped single captures | 8 of 10 right; 500 Hz with 0.25, 200 Hz with 0.10 | | |
| NORMal, stopped single captures | 2 of 10 right; 142.86, 125 Hz, invalid | | |

The wrong values are exact fractions of the true ones (500 Hz with duty 0.25, 200 Hz with 0.10): the engine misses edges and takes 2 or 5 periods for one. Its thresholds follow VTOP and VBASe (percent of the amplitude), which move by ~0.7 V with the overshoot of the untrimmed divider: likely cause, not proven. HRESolution helps a lot, but does not remove the outliers. Measuring on a stopped acquisition does not help.

Driver (C2 follow-up): `set_acquire()` reads its setting back; `measure_series()` returns median, min and max of n readings, so a verdict can check the spread.

Not decided: trim CH1 first and measure again; set absolute thresholds (needs the MHO900 programming guide: not on this machine); or take timing from the waveform, as Core4's `analysis.py` does, which departs from "verdicts come from the scope's own measurements" (ADR 0009, #23).
