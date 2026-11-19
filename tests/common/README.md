# tests/common

> The code examples below come from the uSolarVerter application branch (`apf_quadrature`), where these helpers were written; the helpers themselves are board-agnostic.
Shared code for every host test. Nothing here is compiled into the firmware.

| File | Role |
|---|---|
| [`check.h`](check.h) | `tc::Criterion`, `tc::info`, `tc::blocked`, `tc::finish`: per-criterion reporting |
| [`l0_build.sh`](l0_build.sh) | `l0_build()`: compiles and links one L0 test binary |
| [`make_record.py`](make_record.py) | Turns result lines into `record.md` |
| [`review_report.py`](review_report.py) | Writes a campaign's or study's `report.html` (procedure §8.5) |
| [`interactive_plot.py`](interactive_plot.py) | Interactive review figures: zoom in time, read values (uPlot) |
| [`vendor/`](vendor/) | uPlot 1.6.32 (MIT, `uPlot-LICENSE`), inlined into the review pages so they work offline |
| [`catalogue.py`](catalogue.py) | Loads and checks [`../catalogue.json`](../catalogue.json); `--procedure` writes procedure §6 |
| [`control_center.py`](control_center.py) | Writes `tests/out/control-center.html`: where the validation stands, one tab per stage |
| [`scope.py`](scope.py) | Tektronix MSO54 over USB: screenshot, measurements, setup, waveforms (`capture()`, used by `tests/l2/bench.py`; standalone captures go to `tests/out/scope/`) |
| [`rigol_mho.py`](rigol_mho.py) | Rigol MHO984 over Linux usbtmc (SPOC bench): setup, single acquisition, the scope's own measurements, screen waveforms; ported from Core4 `tests/twt_bnc/rigol_scope.py`. Standalone: `python3 tests/common/rigol_mho.py` prints `*IDN?` and CH1..CH4 measurements. Quirks: [docs/safety/spoc_bench.md](../../docs/safety/spoc_bench.md) |
| [`spoc/`](spoc/) | SPOC bench host side: `boards.py` finds the CUT (2fe3:01ff) and MUX (2fe3:0102) shell ports by USB ID (interface 2; interface 0 is the console); `thingset.py`, `mux_controller.py` copied from SPOC_Software (`MuxController.route('ch1', 'PA8')`); `injector.py` drives the MUX Spin's injector on TP5 (`Injector(mux).level(1200)`, `.step(True)`, `.off()`) |
| [`spoc_preflight.sh`](spoc_preflight.sh) | What the SPOC bench needs: both Spins, scope access, scope answering `*IDN?` |
| [`udev/`](udev/) | udev rules giving USB access without root: MSO54, and `99-rigol-usbtmc.rules` for the MHO984 (`sudo cp tests/common/udev/99-rigol-usbtmc.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger`, then replug the scope) |
| [`shim/`](shim/) | Host stand-ins for hardware-specific headers |

## check.h

A `tc::Criterion` collects observations and prints **one** result line when it goes out of scope. A failing observation does not stop the test, so every criterion gets a verdict.

```cpp
{
    tc::Criterion c("L0-BST-02.a", "Leg 1 alone below and at its current limit");
    c.check("leg 2 off", !boost.isLeg2Enabled());          // boolean
    c.near("ILow1 ref", boost.getILow1Ref(), 2.0, 1e-5);   // |measured - expected| <= tol
    c.range("duty", duty, 0.0, 0.95);                      // lo <= measured <= hi
}
tc::info("L0-GFL-02.c", "PLL lock time", t, "s");          // INFO: no limit stated
tc::blocked("L1-APP-01", "IDLE on a healthy grid", "plant model missing");
return tc::finish();                                       // 1 if any criterion failed
```

Result line format, one per criterion:

```
@@<TAB>id<TAB>verdict<TAB>description<TAB>limit<TAB>measured
```

| Verdict | Produced by |
|---|---|
| `PASS` | `Criterion` with at least one observation and none failed |
| `FAIL` | `Criterion` with a failed observation, or with no observation at all |
| `INFO` | `tc::info` |
| `BLOCKED` | `tc::blocked` |

## l0_build.sh

Sourced by each `tests/l0/<object>/run.sh`:

```bash
source "$(dirname "$0")/../../common/l0_build.sh"
l0_build test_boost \
    --strict "$REPO_ROOT/tests/l0/boost/test_boost.cpp" "$REPO_ROOT/src/boost_converter/boost.cpp" \
    --lib    "$LIB_ROOT/control_library/src/pid.cpp"
"$L0_BIN/test_boost"
```

| Option | Flags | Use for |
|---|---|---|
| `--strict` | `-Wall -Wextra -Werror` | Tests and firmware sources (`src/`) |
| `--lib` | `-w` | Third-party libraries in `owntech/lib/USB/` |

`SINGLE_PHASE_INVERTER_DIR` selects another `single_phase_inverter` checkout (e.g. a fix branch); `l0_build.sh` exports it as `INV_LIB_DIR`.

All sources build with `-fsanitize=undefined -fno-sanitize-recover`: undefined behaviour aborts the binary, and `run_all.sh` records that as a `FAIL`. `CXX` overrides the compiler. The build folder is temporary and removed on exit.

## make_record.py

```bash
python3 tests/common/make_record.py --out DIR --levels L0,L1 LOG [LOG ...]
```

Reads every `@@` line from the logs and the [findings register](../../docs/testing/test-procedure.md#11-findings-register), and writes `DIR/record.md`: front-matter (commits, toolchain, counts, verdict), setup table, verdict chart, test-case table, results table (`BLOCKED` first, then `FAIL`), raw-data list and, on failure, the follow-up reminder. A `FAIL` on a criterion listed in an open finding is marked *known* with its issue, any other `FAIL` **new**; an open finding whose criteria all pass is listed under `findings_to_close`. Exit code 1 if any criterion is `FAIL` (known or new) or none was reported. Standard library only.

`SINGLE_PHASE_INVERTER_DIR`, if set, is the library checkout under test (same variable as `l0_build.sh`); the record stamps its commit.

## shim/

| Header | Stands in for |
|---|---|
| `arm_math.h`, `arm_math_types.h` | CMSIS-DSP types, `PI`, `arm_sin_f32`, `arm_cos_f32`, FIR declarations |
| `my_types.h` | Included by `single_phase_inverter` on non-ARM builds |
| `zephyr/logging/log.h` | Zephyr logging; every `LOG_*` expands to nothing |

Add a stand-in only for a symbol the code under test actually uses, and keep it behaviourally equivalent (e.g. `arm_sin_f32` → `sinf`).
