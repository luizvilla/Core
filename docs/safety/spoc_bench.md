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
| C12 | `test(safety): CPU-dead cases` | L0-CPU (`tests/l0/spoc_dead`): lockup per PWM pin on the scope (L0-SUP-08.c); each hardware trip with the CPU halted over SWD, the threshold crossed by an SWD DAC write |
| C13 | `test(safety): comparator checks on the scope` | Trip latency from the GPIO step; low-bound comparator in service. Done 2026-11-23 (L0-LAT, direct probes): comparator and watchdog latency on the pins; the injector's DAC step replaces the GPIO step (its 0 V idle would trip the low bound) |
| C14 | `test(safety): edge cases` | Below threshold; glitch shorter and longer than the filter; fault while arming, during clear, during dead time; two faults at once. Done 2026-11-20 (L0-EDG): arming with a limit exceeded, clear while exceeded, repeated fault, two faults, no trip just inside, comparator glitch filter. Dead time done 2026-11-23 (L0-LAT-05) |
| C15 | `docs(safety): L0 SPOC campaign results` | Campaign under `tests/out/`, control center checked, results here, in the ADR and on the issues; closes #23. Done 2026-11-23 |

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

Firmware: `owntech-foundation/SPOC_Software`, branch `mux-injector` (from `Core-MUX` `295a3f6`), **pinned at `fe04fe3ccb2a59565d297e667cc5be2a22b95283`** (`b500a4e` until 2026-11-20, then the pulse mode). ThingSet `/Mux/Inj`: `wSource` (0 off, 1 DAC, 2 GPIO), `wLevel_mV`, `wGpio`, `xApply`; read back `rSource`, `rLevel_mV`, `rVref_mV`, `rGpio`; pulse mode `wPulseHigh_mV`, `wPulse_ns`, `xPulse`, read back `rPulse_ns`, `rPulseRc`. Host side: `tests/common/spoc/injector.py`.

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

### 2026-11-20, C12: L0-CPU, the protection with the CPU dead

`tests/l0/spoc_dead` (README there), campaign `2026-11-20_52d5d0e-dirty_015727`: 14 PASS, 2 BLOCKED (TP5–TP7).

**L0-CPU-01, lockup per PWM pin (L0-SUP-08.c).** `SAFETY_SUP` on the CUT (bare Spin, no trips, PWM A, C, D, E, F at 200 kHz); its lockup case raises PA6 a few cycles before FAULTMASK and the fault. Marker on SPOC CH1, the pin on CH3, one armed single capture per pin (CH1 rising at 2.5 V: PWM crosstalk on CH1 reaches about 1.9 V and fired a 1.65 V trigger before the lockup in the first run).

- **Finding: without a hardware trip, the lockup did not stop the outputs.** Every pin kept switching after the marker until the watchdog reset, about 1 ms later. With the PWM running, `OUTxR` fault states were Inactive and `SYSCFG_CFGR2` = 0xF (lockup, SRAM parity, ECC, PVD connected), but no timer listened to a fault channel (`FLTxR` = lock bit only). RM0440 28.3.17: the system fault is enabled "as long as the output is protected by the fault mechanism". Experiments: `FLT1EN` on every timer alone, set before the HRTIM driver's timer init, changed nothing; an enabled fault channel (never active) with every timer listening, set after the init, stopped all ten pins within 28 ns.
- **Ruled out: the ST-Link's halting-debug state.** On a power-cycled CUT, with no SWD access, a lockup over USB (`SAFETY_L0_TWIST`, `/Fault` case 5; reference: the reset, seen as the DAC-held I1 node collapsing) stopped PA8 about 1.6 ms before the reset; the same after an SWD connection (`DHCSR.C_DEBUGEN` = 1): 1.5 ms.
- **Fix (`52d5d0e4`):** when no trip uses a fault input, arming enables a sentinel: FLT1 from its pin, active high, locked; every timer listens to it. Without the pin's HRTIM alternate function the input reads low: no FLT1 flag over a PWM run with USB enumerated (PA12, the FLT1 pin, is USB D+); not tested under bulk USB traffic.
- **After the fix:** all ten pins low within 27 ns of the marker (most already low at it and staying low). Boards with hardware trips (Twist, Ownverter) were not affected: every timer listens to the trip channels (shown for PA8 on `SAFETY_L0_TWIST` by the reset-reference capture).
- L0-SUP regression after the fix (`020726`): 36 PASS, 3 INFO; L0-SUP-08.c now points to L0-CPU-01.

**L0-CPU-02, trips with the CPU halted.** `SAFETY_L0_TWIST`, armed as L0-TRP; then in one SWD session: halt (watchdog frozen in debug, HRTIM running), DAC2 DHR12R1 written 50 mV past the threshold, registers and pins read, resume. The PC is identical before and after: no instruction ran.

| Path | DAC | FLT flag | OENR | PWM pins |
|---|---|---|---|---|
| I1_LOW high, COMP1 → FLT4 | 1750 mV | FLT4 set | 0x033 → 0x000 | all low |
| I1_LOW low, ADC1 AWD1 → FLT1 | 250 mV | FLT1 set | 0x033 → 0x000 | all low |
| I2_LOW high, COMP3 → FLT5 | 1750 mV | FLT5 set | 0x033 → 0x000 | all low |
| I2_LOW low, ADC2 AWD3 → FLT6 | 246 mV | FLT6 set | 0x033 → 0x000 | all low |
| V1_LOW high, V_HIGH high (watchdogs) | | | | BLOCKED: TP5–TP7 |

Bench notes: the SPOC preflight takes `SPOC_ROLES=mux` for a test that flashes the CUT itself. The supervision firmware's backup registers survive a flash: a fault latched by another firmware refuses its PWM until cleared. HRTIM timer registers: `OUTxR` at 0x64, `FLTxR` at 0x68; `SYSCFG_CFGR2` at 0x1C (an earlier read at wrong offsets suggested unarmed registers; corrected).

### 2026-11-20, C14 (part): L0-EDG, edge cases

`tests/l0/spoc_edge` (README there), campaign `2026-11-20_18164c6-dirty_025218`: 5 PASS, 1 INFO; with the glitch case, `2026-11-20_81dfd6a-dirty_030645`: 6 PASS, 1 INFO.

| Case | Result |
|---|---|
| 01 first PWM start with I1 at 1750 mV (limit 1700 mV) | refused, outputs never on |
| 02 clear while I1 is still above its limit, then at 0 A | refused (-1), fault kept; then accepted (0) |
| 04 I1, then I2, above their high limits | FLT4 and FLT5 both recorded (tripped 0x39), outputs off |
| 05 I1 20 mV inside each threshold, 10 s with the PWM on | no trip (1680 mV against 1700.0; 320 mV against 299.8) |
| 03 second fault (I2, from the MUX injector) about 70 ms after a clear, next clear about 0.6 s later | INFO: the second fault latched; the next clear was accepted |

**Open: the repeat-clear rule.** ADR 0009 states it two ways. The decision text: "a repeated clear within N s needs a reboot", which is what `safety_supervision_clear_fault()` does (it compares the new clear with the previous one). The implementation note and the Kconfig prompt: "a second fault within N s of a clear". With the code as it is, a fault 70 ms after a clear, cleared 0.6 s later, is accepted; in production (10 s) a fault 2 s after a clear, cleared 20 s later, is accepted too. To decide; the second reading needs the fault time recorded at the latch, which runs in a zero-latency interrupt (no kernel call): stamped by the self-test work within 10 ms, for example.

**Glitch filter (L0-EDG-06, campaign `2026-11-20_81dfd6a-dirty_030645`).** Core-MUX got a pulse mode on the injector's DAC source (`/Mux/Inj` `wPulseHigh_mV`, `wPulse_ns`, `xPulse`; SPOC_Software `mux-injector` commit `fe04fe3`, pushed and pinned 2026-11-20). I2 is held at 1.0 V and pulsed to 2.0 V; each pulse is captured on CH3 at 200 ns/div and its time above the 1.70 V threshold read from the waveform (the firmware's own timing runs long, in steps of about 41 ns). 19 pulses: no trip up to 270 ns above the threshold, a COMP3 → FLT5 trip from 306 ns; the 282 ns filter (fHRTIM/8, N = 6) lies in between. PASS (no trip up to 242 ns, trip from 322 ns). The ADC watchdog on I2 (no filter, sampled every PWM period) caught none of these pulses.

Faults during dead time stay open: they need the fault aligned on the PWM edges, with direct probes.

### 2026-11-20, Ownverter 1.1.0 on the SPOC bench (L0-TRP)

Env `SAFETY_L0_OWNVERTER` (same app as the Twist, Ownverter 1.1.0 devicetree); `tests/l0/spoc_trip/run.sh --run --board=ownverter`. L0-TRP IDs now carry the board (`L0-TRP-02.ownverter.I1_LOW.high`); the Twist campaign was run again under them (`035111`: 6 PASS, 8 BLOCKED, same trip levels). Ownverter campaign `2026-11-20_24545e7-dirty_034118`: 6 PASS, 10 BLOCKED (TP5–TP7).

| Path | Threshold | Trip |
|---|---|---|
| test build manifest vs production Ownverter 1.1.0 | | identical but for reserved DAC2_CH1 |
| I1_LOW high, COMP3 → FLT5 | 1700.0 mV | 1698 mV (−2.0) |
| I1_LOW low, ADC2 AWD3 → FLT6 | 296.0 mV | 316 mV (+20.0) |
| I3_LOW high, COMP6 → FLT3 | 1102.5 mV | 1102 mV (−0.2) |
| I3_LOW low, ADC1 AWD1 → FLT1 | 946.2 mV | 958 mV (+12.2) |

**Finding to confirm with the hardware team: I3_LOW's scaling.** On Ownverter 0.9.0, 1.0.0 and 1.1.0, I3_LOW has gain 0.045 and offset −92.2, the values of the voltage sensors V1–V3_LOW; I1_LOW and I2_LOW have 0.005 and −10. With it, I3's ±7 A limits are a 157 mV window around 1024.5 mV (±156 counts): if the real sensor is like I1 and I2, that is about ±0.8 A of real current, and the board would trip at low current. Present in the overlays since 2024 (`f1ff9600`); not changed here. Open as luizvilla/Core#28: Luiz goes through the calibration history for the true values. The runner now starts a case at 40 % of a narrow window instead of 150 mV, which landed past I3's other bound.

**Observation: the low-bound watchdog trips come early.** Every ADC-watchdog low bound near 300 mV trips 12–22 mV above its threshold (Twist I1 +14, I2 +13; Ownverter I1 +20, I3 +12 at 946 mV), while the comparators trip within 2 mV. The CUT's DAC is linear down to 200 mV (scope, relative to its 1000 mV reading), and the ADCs are calibrated at start. Within the ±25 mV tolerance; to look at with the CUT's raw ADC reading of the pin against the DAC.

The two held pins of the Ownverter have different 0 A levels (I1 1000 mV, I3 1024.5 mV): the runner holds them at their mean when they are less than 50 mV apart (with TP5–TP7).

### 2026-11-20, campaigns with TP5–TP7 wired (C11, C12 complete)

Luiz Villa soldered TP5–TP7; the runner found the wire on CH4 (injector level 1.02 V on CH3 and CH4). Every case now runs on the production builds, nothing BLOCKED:

| Campaign | Result |
|---|---|
| L0-TRP Twist 1.4.2 (`113733`) | 14 PASS: voltage watchdogs V1_LOW high +7 mV, V_HIGH high +6 mV; software paths within +4 to +8 mV, V_HIGH low +23 mV |
| L0-TRP Ownverter 1.1.0 (`112654`) | 15 PASS, 1 FAIL: V_HIGH low (software) +29 mV against ±25 mV, the ADC offset below |
| L0-CPU (`114506`) | 16 PASS: the two voltage watchdogs trip with the CPU halted too |
| L0-EDG (`113300`) | 11 PASS (03 judged under reading B; 07, 08.a/b new) |
| L0-SUP regression (`114939`) | 36 PASS, 3 INFO |

The supervision test build (SAFETY_SUP) now has a 100 ms repeat-clear window like the other test builds: its cases follow each other within seconds, each ending in a clear, and under reading B the next lockup fell inside the 10 s production window.

### 2026-11-20, finding: the frozen-sensor check is off when the PWM starts before data acquisition

Looking at the early low-bound trips, the test firmware got an `/Adc` readout of a channel's raw sample through `DataAPI::peekAdcRaw()`, the call the frozen-sensor plausibility check uses. With the PWM armed, the watchdogs working and the control task running, it returns 0xFFFF ("no value") on every channel. Cause, from the code: when the first PWM output comes before data acquisition, the Safety API starts acquisition itself (`startUnattended()`: DMA, externally triggered dispatch); the control task started afterwards sees acquisition already running and does not take the dispatch (`do_data_dispatch` stays false in `scheduling_start_uninterruptible_synchronous_task`). Nothing dispatches, every peek returns no value, and the frozen-sensor check skips every period without saying so (IEC 60730-1 H.2 7.2 plausibility off). The hardware trips are not affected: the watchdogs and comparators work on the ADCs and comparators directly. The usual application order (control task first, which starts acquisition and dispatches, then the PWM) is likely fine; not measured. To decide: have the control task dispatch after an unattended start, or make the Safety API refuse or report that order. The ADC-against-DAC comparison for the early low-bound trips waits for it.

**Decided and fixed (2026-11-20, Luiz Villa: both).** (a) The control task takes the dispatch over when it finds acquisition started by the Safety API (`DataAPI::startedUnattended()`), whatever `manage_data_acquisition` says. (b) A watched current sensor with no reading for `CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS` periods while the PWM runs latches a self-test fault ("no current reading (data not dispatched)", `SAFETY_SELFTEST_NO_DATA`), so the check can no longer be off silently. Bench checks: L0-EDG-07 (raw reading follows the DAC after an unattended start), L0-EDG-08 (dispatch stopped by a test hook, `/Fault` case 17). Limit left: with no control task at all the plausibility check does not run (nothing calls it); the hardware trips and the kernel-timer watchdog feed still do.

**Second bug behind it: the dispatch buffer overflowed (Spin API, data_dispatch.cpp, since V1).** With the dispatch working, the first campaign tripped "frozen current sensor" during the first arming, and the raw reading did not follow the DAC (1986 counts at 1000, 1100 and 1200 mV). In task dispatch, each channel collects its samples in a 32-slot buffer that only `data.get*()` empties. A channel the application never fetches (it only peeks, or does not read it) filled up after 32 samples; the count stopped at 32, every new sample was written at index 32, one past the end of the 64-byte heap block, every period, and `peek` returned slot 31 for ever. Any application with an enabled channel it does not fetch each period wrote past a heap block at the task rate. Fixed: a full buffer keeps its last slot for the newest sample. After the fix the raw reading follows the DAC (2 counts/mV, ±1–2 counts of noise) and the frozen-sensor check stays quiet on the held pins.

**Third and fourth bugs behind it (Ownverter 1.1.0).** (1) *Aliasing.* The task finds the new samples from the DMA position modulo the buffer. The unattended start sized the buffer for 16 conversions, rounded to whole sequences: 20 slots for the Ownverter's 5-channel ADC1, and a 100 µs task at 200 kHz brings exactly 20 conversions per period (one per PWM period, discontinuous mode), which read as none: I3's reading stayed at 2058 counts while its pin moved, and the frozen-sensor check tripped at the first arming. The Twist's 3-channel ADC1 (18 slots) updated by luck. The unattended buffer now holds 64 conversions (task periods up to 63 PWM periods); the task prints a warning at takeover when its period needs more, and stale readings beyond trip the frozen-sensor check (fails safe). (2) *Allocation not checked.* A first try with 512 conversions ran out of the 4 KB heap (`CONFIG_HEAP_MEM_POOL_SIZE`): `k_malloc` returned NULL for ADC2 and the DMA was programmed to write its samples at address 0 (CMAR = 0, read over SWD); V3_LOW and I2_LOW's software checks saw a constant 2048 and never tripped. `data_dispatch_init` now leaves an ADC without memory out (a message, no DMA, "no value", which the plausibility check reports).

**Finding: the CUT's ADC reads about 14 mV low at the bottom of its range.** With `/Adc`, V_HIGH (Ownverter, ADC1 channel 8) reads 286 mV with 302 mV on the pin (scope CH2, CUT DAC at 300 mV, which is linear down to 120 mV within 2 mV); at 1000 mV the current pins read 993–995 mV. A fit gives a slope of 0.99 and an offset of about −14 mV. It explains both earlier observations: the ADC-watchdog low bounds trip 12–22 mV early, and the software low bounds trip late (Twist V_HIGH low +23 mV, Ownverter +29 mV at 167 mV, outside ±25 mV: L0-TRP-03.ownverter.V_HIGH.low FAIL). Not explained yet: sampling time against the bench's source impedance (MUX switches, SPOC dividers) or the ADC's own offset. To measure: the offset against the level and the sampling time, on a pin driven straight by the DAC.

**Measured (2026-11-20, L0-ADC, `tests/l0/spoc_adc`, campaign `2026-11-20_51c9ef7-dirty_133524_Mc6XL1`): the ADC's own offset is −5 mV; the rest came from sampling on the switching edges.** The CUT DAC and ADCs share the VREFBUF, so ADC counts − DAC code is the error whatever VREF's true value (VREFINT puts VREF+ at 2048.7 mV). A test probe (`/AdcJ`, injected conversions with a chosen sampling time) and per-channel statistics in the control task (`/Adc xStat`) gave:

| Reading | Error (counts; 1 count = 0.5 mV) |
|---|---|
| DAC pin PA6 on ADC2, no SPOC path, 100–1900 mV | −7 to −11, fit −10 counts, gain +0.06 % |
| I1 (PA1) and V_HIGH (PC3) through the SPOC path, each ADC | −8 to −11: the path adds ≤ 1.5 counts |
| Against the sampling time, 6.5 to 640.5 cycles | flat: not the source impedance |
| Right after VREFINT on the sampling capacitor, 12.5 cycles | +3 at 100 mV, 0 above: no memory worth noting |
| Production reading (regular group), V_HIGH, PWM outputs off | −8 to −10 at every level |
| Same, outputs on | **−61 at 100–150 mV, −43 at 200, −15 at 300**, −5 at 500, −11 to −13 at 1000–1500 |
| Injected conversions at random instants, outputs on | mean −9 (as off); single samples −105 to +185 |
| Trigger moved to a leg's CMP3, swept over the period, outputs on | −10 away from the edges; +12 to −83 within ~0.06 of an edge (0 and 0.5 at duty 0.5) |

Mechanism: in this build both ADCs are triggered by the HRTIM **master period** (HRTIM ADC1R = ADC3R = 0x10, `ADxMPER`; ADC1 EXTSEL 21 `hrtim_adc_trg1`, ADC2 22 `hrtim_adc_trg3`, read over SWD). The Safety API gives a software-triggered ADC its HRTIM trigger (`safety_data.cpp`) and nothing moves it to a leg; the master period is the legs' rising edge. Each edge puts a ±90 mV spike on the pin node, settled within 0.5 µs (CH2, 20 mV/div); every regular sample lands in it, at a level-dependent point of the transient. The ADC-watchdog low bounds (single samples) and the software low bounds near 170–300 mV read through it: the early watchdog trips (+12 to +22 mV) and L0-TRP-03.ownverter.V_HIGH.low (+29 mV) come from the sample instant, not from the ADC. Away from the edges the ADC is 5 mV low against the DAC (the comparators, whose thresholds are DACs, trip within 2 mV): within the ±25 mV tolerance.

How the spike reaches the pin on the bench (SPOC mux crosstalk, or the CUT's own I/O) is not separated.

**Correction (2026-11-20, Luiz Villa): the edge sampling came from the test firmware, not from the boards.** `tests/firmware/safety_l0` drove the legs through `spin.pwm` alone (`initFixedFrequency`, `initUnit`), so they ran left-aligned, the driver default, with the ADCs on the master period. Twist 1.4.2 and Ownverter 1.1.0 set their legs up through `shield.power` from the devicetree: up-down counting (`default-modulation = "UpDwn"`), `EdgeTrigger_up`, ADC1 from PWMA and ADC2 from PWMC, CMP3 at 0.06 × period (`PwmHAL::enableAdcTrigger`; the shift covers the instrumentation delay). In up-down mode that instant is far from the edges. The test firmware now does the same: by default (`/Pwm wShield true`) each unit that is a devicetree power leg gets its modulation, edge trigger, ADC and decimation as `PowerAPI::initMode` sets them; `wShield false` keeps the old set-up for comparison.

L0-ADC re-run with both set-ups (campaign `2026-11-20_77dab25-dirty_145915_6H7TNy`, V_HIGH, production reading, counts):

| Level | Shield set-up, outputs on | off | As found, outputs on | off |
|---|---|---|---|---|
| 100 mV | −9.5 [−12, −7] | −10.0 | −60.6 | −10.3 |
| 200 mV | −9.4 | −9.8 | −42.6 | −10.1 |
| 300 mV | −9.6 | −10.1 | −14.9 | −10.3 |
| 1000 mV | −8.4 | −8.8 | −11.1 | −9.0 |

With the shield set-up the PWM changes nothing: the reading is the ADC's own −5 mV. The sweep of CMP3 in up-down mode errs only between 0.40 and 0.60 of the period register (the edges at duty 0.5), far from 0.06. The earlier L0-TRP, L0-EDG and L0-CPU campaigns ran with the as-found set-up, and are re-run.

**L0-TRP re-run with the shield set-up (campaigns `2026-11-20_77dab25-dirty_150652_bc41ai` Twist, `..._151121_lHhRoB` Ownverter): every case PASSes (14 Twist, 16 Ownverter); L0-TRP-03.ownverter.V_HIGH.low passes.** Every low bound now trips 4 to 6 mV above its threshold, the ADC's −5 mV offset, where the as-found set-up gave +9 to +29 mV:

| Case | As found | Shield set-up |
|---|---|---|
| Ownverter V_HIGH low (software) | +29 mV, FAIL | +4 mV |
| Twist V_HIGH low (software) | +23 mV | +4 mV |
| Ownverter I1_LOW low (ADC2 AWD3) / I2_LOW low (software) | +18 / +18 mV | +4 / +4 mV |
| Twist I1_LOW low (ADC1 AWD1) / I2_LOW low (ADC2 AWD3) | +12 / +9 mV | +6 / +4 mV |
| Ownverter I3_LOW low (ADC1 AWD1) | +11.2 mV | +5.2 mV |

The high bounds (comparators and watchdogs) are unchanged, within −2 to +6 mV.

**L0-EDG and L0-CPU re-run with the shield set-up (2026-11-23, campaigns `2026-11-23_77dab25-dirty_114008_50zPxN` L0-EDG, `..._114427_qIgccu` L0-CPU, Twist): every case PASSes (10 L0-EDG, 16 L0-CPU), as before (`2026-11-20_6af6bbb-dirty_105929_Rx8QKv`, `2026-11-20_10d8aa6-dirty_114506_QCwkMN`).** The changes are within the bench's noise:

- L0-EDG-06: the longest pulse without a trip is 248 ns (was 246 ns) and the shortest with one 286 ns (unchanged). The ADC watchdog caught none of the 19 pulses this time (it caught some before). It samples once per PWM period, at a different instant in each set-up, so whether it catches a pulse depends on where the sample falls; this does not count toward the verdict.
- L0-EDG-07: the step is 400 counts for 200 mV (was 399).
- L0-CPU-01: the last switching edge falls 0 ns after the marker on every pin (up to 27 ns before).
- L0-CPU-02: every trip is taken in the same instruction of the halted loop (PC 0x0803478a, a new build), with OENR going from 0x033 to 0x000 and the expected FLTx set.

The as-found set-up affected only the ADC-side low bounds, which are measured by L0-TRP.

**Bug found on the way (HRTIM driver, for the PRs at the end):** in up-down mode `hrtim_tu_init()` stores the frequency it computes for the doubled count (`frequency = _period_ckpsc(2 × frequency)`), so each further `initUnit()` on the same unit doubles the PWM frequency again; a few re-inits later the period falls below the minimum and the driver's `assert` aborts (kernel panic, `abort()` at 0x0801cb44). Left-aligned units are not affected. Applications init once; the test firmware re-inited at each PWM start, and now inits each unit once per boot.

Bench note: the MHO984 came back with its channels at ×1 after a replug (readings a tenth of the truth, the glitch case never triggered). The runners now set the four channels themselves (×10, 0.5 V/div) and find the scope's `/dev/usbtmcN` (`rigol_mho.scope_path()`, or `$RIGOL_USBTMC`).

### 2026-11-20, repeat-clear rule decided

Reading B (Luiz Villa): the time runs from a clear to the next fault; `SafetyAPI::rebootRequired()`, the fault text and `/Safety rRebootRequired` tell the operator why a clear is refused. Details in ADR 0009; checked by L0-EDG-03, now judged.

### 2026-11-23, C13 and C14 dead time: L0-LAT, direct probes

`tests/l0/spoc_latency` (README there), campaign `2026-11-23_ffc947f-dirty_133249_rZhctU`, Twist 1.4.2: **5 PASS**.

Direct ×10 probes on the CUT: CH1 on PA8, without a ground spring; CH2 on PA9; CH4 on PC1. The MUX injector's DAC source steps I2 (PC1) from its 0 A level across a threshold, while the CUT DAC holds I1. There were 300 high-bound trips and 30 low-bound trips, asynchronous to the 200 kHz up-down PWM.

| Case | Result | Limit |
|---|---|---|
| L0-LAT-01 dead time of leg 1 on the pins | 100.4 / 100.9 ns; DTxR 0x00880088 | 100 ns ±25 ns |
| L0-LAT-02 I2_LOW high bound, COMP3 → FLT5: from PC1 crossing 1.70 V to the outputs below 0.5 V | 256.6 to 312.1 ns, median 287.6 ns (295 trips) | ≤ 360 ns (31 + 282 filter + 47) |
| L0-LAT-03 I2_LOW low bound, ADC2 AWD3 → FLT6 | 0.72 to 15.28 µs, median 9.69 µs (30 trips) | ≤ 3 × 5 µs + 1 µs = 16 µs |
| L0-LAT-04 every trip | PA8 and PA9 never both high; after the turn-off, at most 0.065 V | < 0.5 V |
| L0-LAT-05 faults in the dead time | 5 of 300; the next output never turned on | ≥ 3 |

- **Comparator path.** The 55 ns spread is one filter sample, from the synchronisation to fHRTIM/8. The time is mostly the 282 ns filter; L0-EDG-06 measured where the filter cuts, and the 31 ns of the comparator itself cannot be separated with the filter on.
  - In 54 of the 300 trips FLT6 also latched: the 2.0 V step is outside I2's ADC watchdog window too, and a conversion sometimes falls before the comparator path acts.
- **Watchdog path: the reaction time follows the ADC sequence.** ADC2 converts one channel of V2_LOW, I2_LOW and I_HIGH per trigger. I2 is therefore sampled every third period, and the latency spreads evenly over three periods.
  - The first run against the ADR's "≤ 3 switching periods" missed it by 159 ns (15.16 µs).
  - That figure assumed a 2-event fault counter, dropped on 2026-11-19.
  - **Decided (Luiz Villa):** the bound is (channels on that ADC) × period + one conversion, taken from each board's manifest. This is in ADR 0009.
- **Fault at the end of a dead time.** When the fault lands just as a dead time ends, the next output starts to turn on and is cut one HRTIM clock later, about 5 ns, with peaks up to 1.64 V.
  - This was seen twice in 600 trips over two campaigns.
  - The cut comes at the normal latency, so it is an ordinary trip: the runner's turn-off is the last moment either output is above 0.5 V.
  - Whether a gate driver reacts to a 5 ns pulse is not checked here; for L2.
- **Test firmware.** The test firmware now applies the devicetree dead time, as `PowerAPI::initMode` does. The L0-TRP, L0-EDG, L0-CPU and L0-ADC campaigns before 2026-11-23 ran without it. The thresholds do not depend on it.
- **Bench notes.**
  - In one run the CUT's USB link dropped and re-enumerated after 132 trips. The firmware kept running, armed with the PWM on. The runner now reopens the port and repeats the trip; there were no drops in the next three runs.
  - The first run reported BLOCKED: the BNC cables were still on CH1 and CH2. The probe check now also requires a 0–3.3 V swing, because a SPOC node shows the switching spikes at 400 kHz.

### 2026-11-23, C15: final L0 campaign

The final campaign is `tests/run_all.sh --l0 --target` (`2026-11-23_ff4df40-dirty_150533_7bVcN7`), on code `ff4df403`. L0-TRP also ran for Ownverter 1.1.0 (`..._154743_UxC1Mv`). Afterwards the CUT got SAFETY_L0_TWIST back.

| Test | Result | Against the previous campaign |
|---|---|---|
| L0-ALC allocation | 34 PASS | same |
| L0-API one Safety API | 34 PASS | same |
| L0-CMP comparator path (bare-Spin build) | 40 PASS, 2 INFO | L0-AWD-01.b updated: AWD1 has no filter since 2026-11-19 (the expectation predated it) |
| L0-SUP CPU supervision | 36 PASS, 3 INFO | same |
| L0-TOV timing overhead | 11 PASS, 11 INFO, 2 FAIL | L0-TOV-03/05.S0_50 task-start jitter, tracked in #27 (decided: does not block #23); L0-TOV-03.S0_100 now passes |
| L0-ADC CUT ADC offset | 25 INFO | −9.5 counts at 100 mV with the PWM on (shield set-up), as on 2026-11-20 |
| L0-CAL bench calibration | 14 PASS, 10 INFO, 3 FAIL; then 17 PASS alone (`..._154535_qkRAg5`) | in the campaign, one invalid FREQuency/PDUTy reading among five failed CH2, CH3, CH4; trims, amplitudes and skews were good on every channel |
| L0-CPU CPU dead | 16 PASS | same |
| L0-EDG edge cases | 10 PASS | glitch filter edge 246 / 284 ns |
| L0-LAT latency, direct probes | BLOCKED (probes off) | result kept from `2026-11-23_ffc947f-dirty_133249_rZhctU`, 5 PASS |
| L0-TRP Twist 1.4.2 | 14 PASS | every trip within 1 mV of 2026-11-20 |
| L0-TRP Ownverter 1.1.0 | 16 PASS | every trip within 1 mV of 2026-11-20 |

On both boards the low bounds trip 4 to 6 mV above their thresholds, which is the CUT ADC offset. The high bounds trip between −2 and +6 mV.

The control center shows each test's newest run. A campaign where a test only reports BLOCKED does not hide the newest run where it ran.

Open after #23:
- #28: I3_LOW calibration on Ownverter (data being gathered).
- #27: L0-TOV jitter.
- The bug PRs: dispatch overflow, aliasing, unchecked `k_malloc`, dispatch takeover, HRTIM up-down re-init.

Bench findings while running C15:
- **SPOC preflight order.** After L0-TOV the CUT runs another firmware. The SPOC tests that flash the CUT now check only the MUX before flashing.
- **Board discovery stalled.** A firmware that never reads its shell port stalled board discovery 30 to 50 s per port, on the kernel's closing wait. `spoc.boards` now discards the unsent probe.
- **The MUX hung once.** Neither of its ports was read; a USB replug brought it back.
- **The scope's FREQuency and PDUTy are occasionally invalid.** In the campaign, one invalid reading in five failed three L0-CAL cases. The same bench passed alone two minutes later. This is the same kind of reading the README links to long cables; if it comes back, check the BNC cables on CH2–CH4 (30 cm).

### 2026-11-23, #28: Ownverter calibration data

These are the production calibration fits of three Ownverter **v1.1.1** boards (Luiz Villa; serials ending 7GLG and YR2P from 2026-06-05, 9SEZ from 2026-11-06). Each fit is a regression of DMM reading against raw ADC counts. They are in the same units as the overlays' `default-gain`/`default-offset`: V_HIGH fits 0.0298–0.0299 against the overlay's 0.02996.

| Sensor | 7GLG gain / offset | YR2P | 9SEZ | Zero (counts) | Overlay v1.1.0 |
|---|---|---|---|---|---|
| I1_LOW | 0.00929 / −19.42 | 0.00916 / −18.93 | 0.01011 / −21.65 | 2066–2141 | 0.005 / −10 |
| I2_LOW | 0.00944 / −19.61 | 0.00928 / −19.30 | 0.00940 / −19.59 | 2077–2084 | 0.005 / −10 |
| **I3_LOW** | 0.00965 / −20.08 | 0.00928 / −19.30 | 0.00999 / −20.95 | 2080–2097 | **0.045 / −92.2** |
| I_HIGH | −0.00958 / +20.15 | −0.00957 / +20.14 | −0.00899 / +18.90 | 2103–2104 | 0.005 / −10 |
| V1–V3_LOW | 0.0295–0.0308 / −0.98 to +0.65 | | | ≈ 0 | 0.045 / −92.2 |
| V_HIGH | 0.02981 / 0.14 | 0.02992 / 0.26 | 0.02990 / −0.03 | ≈ 0 | 0.02996 / 0 |

The fits had R² of at least 0.98, except V2_LOW on 7GLG and YR2P (0.94).

**I3_LOW is a current sensor like I1 and I2.** Its gain and zero match theirs on every board. The overlay's 0.045 / −92.2 is the voltage sensors' scaling, copied in by mistake, as #28 suspected.

**The overlay does not match these boards on any current sensor, and on no low-side voltage sensor either:**
- **Current gains.** I1/I2/I3 measure 0.0092–0.0101 A per count, about 1.9× the overlay's 0.005.
- **Current zero points.** They sit at 2066–2141 counts, against 2000.
- **I_HIGH sign.** I_HIGH has the opposite sign.
- **Low-side voltages.** V1–V3_LOW read from 0 V (gain 0.030, offset ≈ 0), where the overlay has a scaling centred on 2049 counts (0.045 / −92.2).

**What this means for the trips:**
- The Safety API turns limits into counts from the **devicetree** at build time. Per-board NVS calibration changes only the software readings (SensorsAPI), never the thresholds.
- **I1/I2 limits.** With the overlay's 0.005 / −10, the ±7 A limits become counts 600 and 3400. On these boards that is −13.4 to −15.6 A and +12.2 to +12.7 A of real current: about 1.8× the intended limit.
- **I3 limits.** The same ±7 A becomes counts 1893–2205, which is −1.7 to −2.0 A and +1.1 to +1.2 A real. Phase 3 would trip at low current.

The bench measured the thresholds correctly (L0-TRP), but the thresholds come from the wrong scaling.

**Hardware history (OWNVERTER KiCad repository: v1.0.0 `4ff54ed`, v1.1.0 as built `29fcf13`, v1.1.1 `6b4b9be`).**

| Sensing | v1.0.0 | v1.1.0 | v1.1.1 |
|---|---|---|---|
| Leg current I1–I3 | ACS730KLCTR-20AB | ACS724LLCTR-50AB, new op-amp network | as v1.1.0 |
| Leg voltage V1–V3 | AMC1100 | AMC1311B, new dividers | as v1.1.0 |
| High side | ACS723-20AB, AMC1100 | ACS723-20AB, AMC1311B | as v1.1.0 |

- **v1.1.0 to v1.1.1:** in the sensing sheets, only renumbered parts (U902 → U607, R824 → R625), Schottky diodes, one added decoupling capacitor and a connector changed. Every sensing part and value is the same.
- **The overlays:** `ownverter_v1_1_0.overlay` has exactly the scaling of `ownverter_v1_0_0.overlay`, which fits v1.0.0's parts:
  - 0.005 A/count is 100 mV/A at the ADC, which fits the ACS730-20AB;
  - a scaling centred on mid-scale fits the AMC1100's differential output.
- **Conclusion:** the v1.1.0 overlay was never updated for the v1.1 sensing. The v1.1.1 calibrations hold for v1.1.0, and no v1.1.1 overlay is needed.

Mean of the three boards, for the v1.1.0 overlay:

| Sensor | Gain | Offset |
|---|---|---|
| I1/I2/I3_LOW | 0.00951 | −19.87 (zero 2089 counts) |
| I_HIGH | −0.00938 | +19.73 |
| V1–V3_LOW | 0.03002 | −0.02 |
| V_HIGH | 0.02996 | 0, unchanged |

Board 9SEZ's I1 sits 7 % above the mean.

**Fixes:**
- **I3_LOW** takes the I1/I2 scaling in every overlay.
- **v1.1.0** takes the v1.1 values above, which also covers v1.1.1 boards.
- **Not checked:** v0.9.0's hardware.

### 2026-11-24, #28: Ownverter overlays fixed, re-run on the bench

Commits `b4ae382a` (overlays, allocator) and `d18e8bdb` (flash CRC, runner).

**Overlays.**
- `ownverter_v1_1_0.overlay`: the calibration means above. I1–I3's ±7 A limits move from counts 600/3400 (about ±13 A real on v1.1 boards) to 1353/2825; 0 A is at 2089 counts.
- `ownverter_v0_9_0`, `v1_0_0`: I3_LOW takes I1/I2's 0.005 / −10.

**Allocator.** `safety_allocation.py` assumed counts rise with the value. A negative gain (v1.1 I_HIGH) would swap the bounds, give the comparator the wrong polarity and invert the watchdog window. A hardware-protected limit on a negative-gain sensor is now a build error; L0-ALC-16 checks it. No Ownverter limit is on I_HIGH. Supporting such sensors (swap the bounds, invert the comparator, from the gain's sign) is left for when a board needs it.

L0-ALC campaign `2026-11-24_3fcff75-dirty_110036_BYrXdw`: all PASS, every shield builds; the committed protection tables are unchanged (they list paths, not counts).

**Finding: the run-time flash CRC covered slot 1 and the NVS partition.** The first L0-TRP run on the new build (`..._110347_s94fnl`, repeated in `..._110852_npKpID`) failed every case with "self-test failed: flash CRC; clear: reboot".
- The run-time self-test compares a CRC of `__rom_region_start..__rom_region_end` with the reference taken at boot. With MCUboot, `__rom_region_end` is the end of flash (0x08080000): 448 KB including slot 1 and the `storage` partition at 0x0807F000.
- At the first data start after a reflash, NVS was written (178 bytes in `storage` afterwards; most likely NVS's first-use version record). The CRC no longer matched and the fault latched until reboot.
- Proof: after a reset, a run without reflashing (`--no-flash`, diagnostic) armed and tripped normally, and the `storage` page did not change.
- The same would happen on a board in the field after a calibration store, a metadata write or a DFU download to slot 1.
- **Fix:** the CRC covers `_flash_used` bytes, the image (115 KB for SAFETY_SUP, 207 KB for SAFETY_L0_OWNVERTER). New criterion L0-STK-01.e checks `flash_bytes` against the ELF's `_flash_used`.
- L0-SUP/STK campaign `..._111833_nNsPl0`: all PASS, including L0-STK-01.e (115 292 B) and the injected flash fault.
- Not reproduced after the fix: the CUT's `storage` already held its NVS records. The size check shows the partition is out of the CRC.

**Finding: the v1.1 V1–V3_LOW low limit (−10 V) is out of the sensor's range.** With the v1.1 scaling these sensors read from 0 V, so −10 V is −166 mV on the pin and can never trip. L0-TRP now reports a threshold outside 0..VREF as INFO instead of FAIL. Whether the limit should move (for example to 0 V or be dropped) is a decision for the shield's thresholds; a low voltage is a range check, not a hazard (ADR 0009).

**L0-TRP re-runs, after the fix:**

| Campaign | Result |
|---|---|
| Ownverter 1.1.0 (`2026-11-24_d18e8bd-dirty_112037_0QYueR`) | 13 PASS, 3 INFO (V1–V3_LOW low, out of range) |
| Twist 1.4.2 (`..._112459_klI9bL`) | 14 PASS; the CUT is back on SAFETY_L0_TWIST |

Ownverter trips with the new scaling, against the thresholds from the manifest (CUT VREF 2048 mV):

| Path | Threshold | Trip |
|---|---|---|
| I1_LOW high, COMP3 → FLT5 | 1412.5 mV | −3 mV |
| I1_LOW low, ADC2 AWD3 → FLT6 | 672.0 mV | +4 mV |
| I2_LOW high, COMP1 → FLT4 | 1412.5 mV | −1 mV |
| I2_LOW low, software | 676.7 mV | +4 mV |
| I3_LOW high, COMP6 → FLT3 | 1412.5 mV | −1 mV |
| I3_LOW low, ADC1 AWD1 → FLT1 | 676.2 mV | +6 mV |
| V1–V3_LOW high, software | 1166.2 mV | +5 to +6 mV |
| V_HIGH high, ADC1 AWD2 → FLT2 / low, software | 1672.0 / 166.9 mV | +3 / +4 mV |

**Upstream (owntech-foundation/Core), waiting for Luiz's go:**
- PRs: data dispatch buffer overflow, unattended-DMA aliasing, unchecked k_malloc in data_dispatch_init, frozen-sensor dispatch takeover, HRTIM up-down re-init; Ownverter I3_LOW scaling (all overlays); Ownverter v1.1.0 sensing values; run-time flash CRC over the image only; negative-gain allocator guard.
- Issues: I_HIGH sign convention, overlay against production NVS calibration; limits on line voltages (V12, V23, V31) for AC faults on the software path.
- Opened 2026-11-24: owntech-foundation/Core#171 (Ownverter overlays, branch `fix/ownverter-sensor-scaling` on luizvilla/Core), issues owntech-foundation/Core#169 (I_HIGH sign) and #170 (line-voltage limits). The flash CRC fix and the negative-gain guard touch code that is only on this branch: they go up with the Safety API.
- 2026-11-25: owntech-foundation/Core#172 (HRTIM up-down re-init) and #173 (dispatch buffer overflow) opened, each with a test on upstream code. The dispatch takeover and the unattended-DMA aliasing are in code only on this branch: they go up with the Safety API.
- **Found while testing on upstream code (2026-11-25):** (1) `createCritical()` accepts periods its source cannot produce: on the HRTIM source the 8-bit repetition counter wraps above 256 PWM periods (5 ms at 200 kHz ran every 1.160 ms), on TIM6 the 16-bit auto-reload truncates above 6553 µs (10 ms gave 3.46 ms). Fix decided by Luiz: refuse with -1 and a message (branch `fix/task-critical-period-range`). (2) The unchecked `k_malloc` does reach a crash upstream: a 5 ms TIM6 task at 200 kHz needs 1002-sample DMA buffers, ADC2's does not fit in the 4 KB heap, and the board faults after the first task period (branch `fix/data-dispatch-dma-alloc`). Both waiting for Luiz's approval of the texts.

### 2026-11-24, #28: decisions on I_HIGH's sign and the V1–V3_LOW low limit (Luiz Villa)

**I_HIGH keeps the old sign convention: gain +0.00938, offset −19.73** (the calibration magnitude). The reading is −I, as with the old 0.005 / −10, and applications negate it: `OWNVERTER/legacy/FOC_speed_hall_sensor` in the examples (branch `new_OWT_examples`) does `I_high = -meas_data` "because of the way hardware sensor is routed". The calibration's own −0.00938 would have given the true sign and made every such application negate it twice. No limit is on I_HIGH, so no trip changes; L0-ALC `2026-11-24_07c5f00-dirty_132308_sl4V5l`: 35 PASS.

Left open (upstream issue): the production NVS calibration stores the negative gain, so a calibrated board reads +I and an uncalibrated one −I. The sign convention should be the same in the overlay and in the calibration.

**V1–V3_LOW keep the −10 V low limit (never trips, L0-TRP INFO).** The schematic (InverterLeg sheet, v1.1.0 `29fcf13`):
- R616 330 kΩ / R617 5.6 kΩ divide the leg node (to DC−) by 0.0167 into the AMC1311B (U601, input 0 to 2 V); an MCP6486 difference amplifier (U604, 27 kΩ, gain 1) feeds the ADC.
- At the 2.048 V ADC reference: 0.5 mV / 0.0167 = 0.0300 V per count, 0 V at 0 counts, as the calibrations found.
- The leg voltage is a wave plus an offset (Vdc/2 + Vac·sin), always between 0 V and Vdc. Below DC− it can go only by a diode drop in the dead time, which the sensor does not see.
- A 0 V low limit would be the idle state (no bus or leg off) and would trip there, not on an overshoot. Any limit below 0 V cannot trip.
- An AC fault is a limit on a line voltage (V12 = V1 − V2, computed by the application), a computed quantity for the software path. Upstream issue.
