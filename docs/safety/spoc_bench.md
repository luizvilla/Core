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
  - the existing L0 firmware (`safety_comparator`, `safety_supervision`) stays as it is; the generic firmware is new;
  - CH3 may be routed while the injector is off (both pins high impedance); keep both Spins powered together.
- **2026-11-19, #23** (Luiz Villa):
  - the generic safety test firmware is controlled over USB with ThingSet (as Core-CUT); an ST-Link on the CUT (STLINK-V3, SWD checked against the CUT's unique ID) flashes it, reads state when the CPU is dead, and debugs;
  - self-injection uses DAC2_CH1 on PA6 (Spin pin 34), reserved through the allocation; the test build refuses to run if the reservation changes any trip against the production manifest (then inject from PA5, DAC1_CH2). PA6 is unused on Twist 1.4.2, Ownverter 1.0.0 and 1.1.0;
  - boards: Twist 1.4.2 first, then Ownverter 1.1.0. O2 is deprecated: not tested;
  - the CUT stays on the safety firmware between campaigns; the firmware has a 1 kHz calibration output on PA8 for L0-CAL.
- **2026-11-19, #23 C11 sources** (Luiz Villa): arming needs the current pins whose low bound is an ADC watchdog held at 0 A (a floating pin fails the self-test); two pins on each board (Twist I1_LOW, I2_LOW; Ownverter 1.1.0 I1_LOW, I3_LOW) and the bench has two independent sources (CUT DAC on CH1/CH2, MUX injector on CH3). Cases on those current limits run on the production build, the two pins held, one of them stepped. Cases on the voltage watchdogs (Twist V_HIGH, V1_LOW high; Ownverter V_HIGH) need a third source and run on a **second build per board**: an overlay widens only the held current limits' low windows so their pins may float; the voltage path under test keeps its production threshold, and the runner refuses to judge unless every other entry of that build's protection manifest matches production. A stated departure, recorded with each such case.
- **2026-11-20, #23 C11 sources, revised** (Luiz Villa): the second build could not run as decided. A floating current pin reads about -4 A at idle and fails the Safety API's idle plausibility check, so arming is refused even with the low windows widened. Instead, **TP5 is wired to TP7**: the MUX injector holds two pins through CH3 and CH4, and every case runs on the production build (no departure, on Twist and Ownverter alike). CH4 then cannot watch a PWM pin; latency checks use a direct probe. The widened build and its overlay were dropped.

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
| C7 | `test(safety): L0-CAL, trim and skew/gain calibration` | Trim the four channels (Core4 trimming flow), each routed alone; then gain per channel and skew CH1 → CHn (CH1 + CHn routed together): stored with the campaign |
| C8 | `docs(safety): SPOC bench README, photo, ADR note` | L0 runner README with the photo; closes #26 |

### Phase 2: generic firmware and L0 runner (#23)

| # | Commit | Content |
|---|---|---|
| C9 | `feat(test): generic safety test firmware, ThingSet control` | `tests/firmware/safety_l0`: arm, PWM, report and clear faults, any shield |
| C10 | `feat(test): software fault injection in the generic firmware` | Stack overflow, hard fault, interrupts masked, watchdog starved, wild register writes after locking |
| C11 | `test(safety): L0 runner, cases from the manifest` | Limit × sign × path, self-injection through TP1–TP3, routes set with the PWM stopped; the current pins with a watchdog low bound held by the injector through CH3 and (TP5–TP7 wired) CH4; production build only, its manifest checked against `tools/safety_check.py` |
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

The scope's bandwidth and sample rate against the 31 ns verdict: see 2026-11-19, C8.

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

**Cause: the BNC cables.** Luiz Villa replaced the 1 m BNC cables between SPOC and scope with 30 cm ones; nothing else changed. Use the 30 cm cables. Same signal, 500 µs/div:

| Setting | FREQuency (10 readings) | PDUTy | VTOP / VBASe |
|---|---|---|---|
| NORMal, running | 1000 Hz (one 999.5) | 0.5 (one 0.50025) | 3.37–3.39 / 0.01 V |
| NORMal, stopped single captures | 1000 Hz ×10 | 0.5 ×10 | |
| HRESolution, running | 999.5–1000 Hz | 0.5–0.50025 | 3.39 / −0.01 V |
| HRESolution, stopped single captures | 999.5–1000 Hz | 0.5–0.50025 | |

The overshoot and the wandering VTOP/VBASe came from the cables, not from the divider trim; the scope's thresholds then missed edges. With good cables the scope's own measurements are usable in either mode, and verdicts stay on them (ADR 0009, #23). The bench README (C8) names the cables to use and the calibration (C7) checks them: a campaign stops if VTOP/VBASe spread or a timing measurement is out of tolerance.

### 2026-11-19, C5 and C6: the injector

Firmware: `owntech-foundation/SPOC_Software`, branch `mux-injector` (from `Core-MUX` `295a3f6`), **pinned at `b500a4e5d37df710018dac232c65e7b8a362d959`**. ThingSet `/Mux/Inj`: `wSource` (0 off, 1 DAC, 2 GPIO), `wLevel_mV`, `wGpio`, `xApply`; read back `rSource`, `rLevel_mV`, `rVref_mV`, `rGpio`. Host side: `tests/common/spoc/injector.py`.

Wiring done by Luiz Villa: MUX Spin pin 2 (PB12) and pin 4 (PB13) to TP5 through their resistors.

| Check | Result |
|---|---|
| DAC full scale (`rVref_mV`) | 2048 mV: VREFBUF is on at 2.048 V on the MUX Spin; levels above that need the GPIO step |
| DAC level at pin 2 / at TP5 (CH3), 0–2000 mV | within ~20 mV of the command (0.52, 1.02, 1.52, 2.02 V for 500…2000 mV) |
| GPIO at pin 4 / TP5, low and high ×2 | 0.03 / 3.30 V; TP5 0.01 / 3.34 V |
| Rising edge at pin 4 (1 GSa/s, 20 ns/div) | under ~6 ns (scope RTIMe 1.6 ns, below what 1 GSa/s resolves) |
| Same edge on CH3 (TP5 through the SPOC divider) | ~20 ns rise, then **5.9 V peak** decaying over tens of µs to 3.34 V: CH3's divider is over-compensated (trimmer too high). TP5 itself is clean (pin 4 on the same node reads 3.3 V). Trim in C7 before any edge is read through CH3 |
| CH3 rule | level change on the same source keeps CH3 routed; a change of source disables CH3 (read back) |
| Status when off | `rLevel_mV` 0, `rGpio` false (fixed in `b500a4e`) |

**Rule relaxed (confirmed by Luiz Villa, 2026-11-19; caveat in ADR 0009: keep both Spins powered together):** ADR 0009 says CH3's mux stays disabled whenever the MUX Spin is not injecting. The firmware forces CH3 off on every change of source, but lets the host route CH3 while the injector is off (both pins high impedance): the C7 calibration routes one CUT pin to all four channels, CH3 included.

### 2026-11-19, C7: trim and L0-CAL

**Trim** (`tests/l0/spoc_cal/trim.py`, Core4's compensation figure, each channel routed alone). Before: CH1 99 %, CH2 no signal (scope CH2 was not cabled to BNC2), CH3 154 % (the over-compensation seen on the injector step), CH4 101 %. Luiz Villa cabled CH2 (a longer cable than the 30 cm ones) and trimmed all four: 99.5, 98.0, 98.3, 100.9 %.

**Findings while building L0-CAL:**
- `MuxController.route()` (SPOC_Test_Software) disables every other route first: routing several channels takes `route_pair()`. An earlier reading "PA8 on all four channels loads the pin" was wrong: only CH4 was routed.
- With TP1–TP3 wired (wire 1), SPOC CH1 and CH2 share one node: routing either shows the signal on both scope channels. Rule: never route two different CUT outputs to CH1 and CH2 while the wire is in, except the intended self-injection source and target.
- The scope's RRDelay read invalid and its FFDelay a constant −0.2 ns at 20 ns/div: skew is taken from the waveforms (50 % crossings, interpolated), as information only.

**Campaign `2026-11-19_f1f2f51-dirty_140237_ugudxj`:** L0-CAL-01 to 05 PASS on all four channels (20 criteria); recorded:

| | CH1 | CH2 | CH3 | CH4 |
|---|---|---|---|---|
| Compensation | 99.3 % | 98.0 % | 98.4 % | 101.1 % |
| VTOP − VBASe | 3.379 V | 3.345 V | 3.362 V | 3.379 V |
| Gain vs CH1 | 1.0000 | 0.9899 | 0.9950 | 1.0000 |
| Skew from CH1, rising / falling | — | +0.73 / +0.42 ns | −4.12 / −4.25 ns | −3.92 / −4.02 ns |

Frequency 1000 Hz (spread ≤ 0.2 Hz) and duty 0.5 (spread ≤ 0.0001) on every channel, CH2's longer cable included. CH3 and CH4 lead CH1 by ~4 ns; the cause (CH1's node carries the TP1–TP3 wire and CH2's divider, or cable lengths) is not separated. A latency verdict read across two channels corrects by this skew.

C8: photographs of the board and the bench in `tests/l0/spoc_cal/README.md` (two of the three photos taken; the third shows a laptop screen and is left out).

**Scope against the 31 ns comparator verdict (#26).** Front panel (bench photograph): MHO984, 800 MHz, 4 GSa/s, 12-bit. 800 MHz gives a rise time of ~0.44 ns (0.35/BW): enough for a 31 ns verdict. The sample rate in use follows the timebase and memory depth: 1 GSa/s at 20 ns/div with the default 10 k points (1 ns per sample, ~3 % of 31 ns). A latency case sets the timebase so that the record runs at ≥ 1 GSa/s, and corrects by the campaign's skew (L0-CAL-07). The SPOC path adds its own delay (mux, divider): the 31 ns verdict itself still uses direct probes (ADR 0009).

### 2026-11-19, C9: generic safety test firmware

`tests/firmware/safety_l0`, env `SAFETY_L0_TWIST` (Twist 1.4.2, production thresholds), flashed through the ST-Link on the CUT. ThingSet on a second USB CDC ACM port (`cdc_acm_uart1`, as Core-CUT; Core2's board has only the console otherwise): `/Safety` (armed, fault, tripped FLTs, outputs, text, clear), `/Pwm` (frequency, duty, units, run), `/Inj` (DAC2_CH1 on PA6), `/Cal` (1 kHz on PA8 from TIM1). Safety API: new `safety-config` property `reserved-dacs`, read into the allocation model (L0-ALC 34/34 PASS after the change).

| Check | Result |
|---|---|
| Protection manifest of the test build against production Twist 1.4.2 | limits, paths, comparator DACs, legs and resources identical; only `reserved: DAC2_CH1` added |
| Boot line | `TWIST v1.4.2 (production): V1_LOW sw-/awd+, V2_LOW sw-/sw+, V_HIGH sw-/awd+, I1_LOW awd-/comp+, I2_LOW awd-/comp+, I_HIGH sw-/sw+` |
| `/Cal` through the four channels | compensation 99.4 / 98.1 / 98.4 / 100.9 % (as with Core-CUT) |
| `/Inj` at PA6, through CH1/CH2 (TP1–TP3) | 0.51, 1.03, 1.54, 2.06 V for 500…2000 mV; full scale 2.048 V |
| PWM start, sensor pins floating | **refused**: `arming failed: ADC2 AWD3 (I2_LOW) not configured` (the watchdog self-test with the real window fails: a floating pin reads −10 A, outside −7…7 A; the message says "not configured" for any failed state) |
| PWM start, I1 (PA1) held at 1.0 V by the CUT DAC (CH1/CH2), I2 (PC1) by the MUX injector (CH3) | **armed** with production thresholds: `armed V1_LOW awd, V_HIGH awd, I1_LOW awd, I2_LOW awd, I1_LOW comp+, I2_LOW comp+ · self-test OK`; outputs TA1, TA2, TC1, TC2 on; stopped cleanly |

Board discovery now asks each Spin's ThingSet root (`Mux` → MUX; `Cut` or `Safety` → CUT): the CUT on this firmware reports 2fe3:0100 like the MUX.

Twist pins for the trips: I1 on PA1 (Spin 30, mux1 input 1), I2 on PC1 (Spin 25, mux3 input 1); 0 A at 1.000 V, window 0.300–1.700 V (−7…7 A).

### 2026-11-19, C10: software fault injection

`/Fault` in the generic firmware (cases in the header of `tests/firmware/safety_l0/main.cpp`). A 100 µs control task (TIM6) now starts with the first PWM start, after arming: it feeds the watchdog and runs the threshold read-back, as in an application. Safety API: `safety_trips_test_regs()` (test hooks only) gives the firmware each trip's registers. Bring-up on Twist 1.4.2, production thresholds, I1/I2 held at 1.0 V (as C9); results read over ThingSet, those that end in a reset after the reboot.

| Case | Result |
|---|---|
| 1 thread stack overflow | reset; fatal error latched, reason 2 (`K_ERR_STACK_CHK_FAIL`) |
| 2 undefined instruction | reset; fatal error latched, reason 36 (`K_ERR_ARM_USAGE_UNDEFINED_INSTRUCTION`), PC in the app |
| 3 every interrupt masked (PRIMASK) | watchdog reset; watchdog fault latched |
| 4 control task never returns | watchdog reset; watchdog fault latched |
| 5 lockup (FAULTMASK + fault) | watchdog reset; watchdog fault latched |
| 6 wild writes to the locked set-up (COMP CSR, FLTINR1/2, FLTxR, CFGR2 locks) | none took effect; no fault |
| 7 comparator DAC ±1 count (I1, I2) | caught by the read-back: 0.19–0.55 ms |
| 8 watchdog window ±1 count (V1_LOW, I1_LOW, I2_LOW) | caught: 0.02–0.29 ms |
| 9 watchdog's external event rerouted (EECR1/2) | **was not caught** (10 ms, write took effect); read-back extended: caught on all four watchdogs, 0.04–0.38 ms |
| 10 timer A OUTxR fault state, outputs on | write did not take effect |
| 11 AWD1 channel changed (CFGR AWD1CH) | **was not caught**; read-back extended: caught, 0.12 ms |
| 12 ADC conversions stopped (ADSTP) | **was not caught** (10 ms); now the read-back latches "watchdog blind": 29 µs |
| 13 a thread spins with `irq_lock()` | **was: no reset, no fault** after 20 s and over minutes in an earlier run (the zero-latency control task kept feeding the watchdog while the kernel, USB and the run-time self-tests were stopped); now: reset, "the system work queue did not run for 20 ms" latched |
| 14 a work item blocks forever on the system work queue | same |
| 15 a work item spins on the system work queue | same |
| 16 `data.stop()` with the outputs on | refused (-2); acquisition and outputs untouched |

Decisions on 12 and 13 (Luiz Villa, 2026-11-19): `data.stop()` refused with the outputs on and the read-back checks the ADC converts; a kernel heartbeat from the self-test work with a 20 ms default timeout (a 1 ms thread at 5 ms was tried first and cost 3 CPU points and 3 µs of jitter in L0-TOV). Why 20 ms and the measured gaps (worst 10.63 ms under load): [kernel_liveness.md](kernel_liveness.md). Regression after the change: cases 2, 3, 4 unchanged; 6–12 as above.

The first case 3 used `irq_lock()` and hung the board this way; it became case 13, and case 3 now masks with PRIMASK. Board discovery (`spoc/boards.py`) now has a write timeout: a hung Spin used to block it.

The scope checks (outputs low within 1 µs of the lockup, L0-SUP-08.c) come with C12/C13; the verdicts with the L0 runner (C11).

### 2026-11-20, C11: L0-TRP, the runner from the manifest

`tests/l0/spoc_trip` (README there). Cases are generated from the test build's protection manifest: one per limit, bound and path. The CUT DAC drives the pin under test through CH1 → TP1–TP3 → CH2; the injector holds the other current pins at 0 A. Each case: arm at 150 mV inside the threshold, quiet 200 ms, 10 mV ramp to the trip, recover, 1 mV ramp; verdict on the fault, the path's FLT, outputs off and the trip level within ±25 mV of the threshold from the manifest counts (DAC, ADCs and comparator DACs share the CUT's VREFBUF).

First campaign (`2026-11-20_99ff3bf-dirty_001306`), TP5–TP7 not wired yet:

| Case | Path | Threshold | Trip | Result |
|---|---|---|---|---|
| L0-TRP-01.a | test build manifest vs production Twist 1.4.2 | | identical but for reserved DAC2_CH1 | PASS |
| L0-TRP-01.b | first arming, I1/I2 held at 1.0 V | | armed, quiet | PASS |
| I1_LOW high | COMP1 → FLT4 | 1700.0 mV | 1698 mV (−2.0) | PASS |
| I1_LOW low | ADC1 AWD1 → FLT1 | 299.8 mV | 316 mV (+16.0) | PASS (after the fix below) |
| I2_LOW high | COMP3 → FLT5 | 1700.0 mV | 1698 mV (−2.0) | PASS |
| I2_LOW low | ADC2 AWD3 (8-bit) → FLT6 | 296.0 mV | 307 mV (+11.0) | PASS |
| V1_LOW high, V_HIGH high (watchdogs); V1_LOW low, V2_LOW both, V_HIGH low, I_HIGH both (software) | | | | BLOCKED: need TP5–TP7 |

**Finding: AWD1 never tripped in service.** The first runs found no I1_LOW low trip even at 200 mV. ADC1 converts a 3-channel sequence (ch2, ch1, ch9); AWD1 watches ch2 alone with its filter at "two consecutive detections" (`TR1.AWDFILT = 1`), and in that sequence it never flagged. With the filter cleared (by SWD, at 250 mV) it tripped at once. The arming self-test converts the channel alone and passed either way, so nothing showed it before. On the Twist, I1_LOW's watchdog bound (low, and high behind COMP1) protected nothing in service. Fixed: no AWD1 filter (AWD2 and AWD3 have none); the read-back now includes the filter bits.

Runner notes: the arming self-tests run once per boot, at the first data start, with the idle plausibility of the current sensors; so the runner resets the CUT and arms first with the held pins at 0 A (driving a current pin to its start level before that fails the idle check). The test firmware now enables the shield's sensors at boot (`enableDefaultSensors()`, as an application) and watches one sensor by name for the software-path cases (`/Sw`). After a flash or reset, board discovery waits up to 60 s for both Spins.

