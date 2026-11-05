# Tests

Host tests of the uSolarVerter firmware. They implement the [test procedure](../docs/testing/test-procedure.md); terms are defined in the [glossary](../GLOSSARY.md).

## Run

```bash
tests/run_all.sh          # every level
tests/run_all.sh --l0     # L0 only, a few seconds
tests/run_all.sh --l1     # L1 only
tests/run_all.sh --help
```

The script prints one line per criterion, writes the test record to `tests/out/<YYYY-MM-DD>_<sha7>[-dirty]/record.md` with one log per test script, and exits non-zero if any criterion is `FAIL`. `tests/out/` is git-ignored: any record can be regenerated from its commit.

```mermaid
flowchart LR
  R[run_all.sh] --> S["tests/l0/*/run.sh<br/>(build + run)"]
  S -- "result lines @@…" --> L[(logs in tests/out/…)]
  L --> M[common/make_record.py]
  M --> D[record.md]
```

Failures already tracked in the procedure's [findings register](../docs/testing/test-procedure.md#11-findings-register) are marked *known* in the record; only **new** failures need an issue. To test a library fix before it is merged:

```bash
SINGLE_PHASE_INVERTER_DIR=~/code/single_phase_inverter tests/run_all.sh --l0
```

## Requirements

| Item | Needed for |
|---|---|
| `g++` with C++17 and UBSan | L0 |
| `python3` ≥ 3.10 (standard library only) | Writing the record |
| A PlatformIO build of the USB environment, once | Fetches `control_library` and `single_phase_inverter` into `owntech/lib/USB/` |
| `.venv` with [`l1/requirements.txt`](l1/requirements.txt) (`pulsim==2.0.0`) | L1; without it L1 is reported `BLOCKED` |

## Layout

| Path | Content |
|---|---|
| [`run_all.sh`](run_all.sh) | Campaign runner |
| [`common/`](common/README.md) | Shared check helper, build helper, hardware stand-ins, record writer |
| [`l0/boost/`](l0/boost/README.md) | L0-BST: boost converter controller |
| [`l0/grid_observer/`](l0/grid_observer/README.md) | L0-GOB: grid observer |
| [`l0/inverter_following/`](l0/inverter_following/README.md) | L0-GFL: grid-following inverter library |
| [`l1/`](l1/README.md) | L1: firmware block in Pulsim; L1-FWB-01 now, plant model and L1-APP to come |
| `out/` | Generated test records (git-ignored) |

## Adding a test

1. Add the test case to the catalogue in the [procedure](../docs/testing/test-procedure.md#6-test-catalogue), with its source and limit.
2. Write it in the matching folder, one `tc::Criterion` per criterion ID (see [common/README.md](common/README.md)).
3. Follow the documentation rules in procedure §7.3: README, license header, Doxygen comments, `@test` tag.
4. A new test folder needs `run.sh` and `README.md`; `run_all.sh` picks it up automatically. Add its object code to `object_code()` in `run_all.sh`.
