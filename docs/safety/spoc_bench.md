# SPOC safety bench (L0)

The SPOC bench runs the Safety API's L0 tests ([ADR 0009](../adr/0009-safety-api-hardware-protection.md)) on a bare Spin with a scope. This is a living document: the plan, the decisions, then the results as each step lands.

Issues: luizvilla/Core#26 (bench), luizvilla/Core#23 (generic firmware and L0 runner) · tracking: luizvilla/Core#14

## Bench

| Item | Identification | Role |
|---|---|---|
| CUT Spin | USB 2fe3:01ff | Card under test: runs the test firmware |
| MUX Spin | USB 2fe3:0102 | Routes CUT pins to the four scope channels; also the independent injector (TP5) |
| Rigol MHO984 | USB 1ab1:0452, `/dev/usbtmc0` | Measures; verdicts come from its own measurements |

Ports are found by USB product ID, never by a fixed `ttyACM` number. Wiring: ADR 0009, implementation note of 2026-11-17.

### Scope link (MHO984 over Linux usbtmc)

Found on the Core4 `test/twt-bnc` bench (`tests/twt_bnc/README.md` there, fw 00.01.00) and seen again here on 2026-11-18:

- The first reply after plugging the scope in (or after a USBTMC clear) is lost. The driver sends `*IDN?` until one comes back and never sends a clear.
- Once the kernel driver is out of step with the scope, every query times out until the USB cable is replugged.
- A screenshot (~130 kB) puts the driver out of step: screenshots are off by default.
- The scope never answers `:SYSTem:ERRor?` or `*ESR?`: setups are checked by reading them back.
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
