# Tests

Board-agnostic tests of the OwnTech Core. Every test is defined once in [`catalogue.json`](catalogue.json); results come only from campaigns under `tests/out/`, and [`tests/out/control-center.html`](out/control-center.html) shows where the validation stands. Terms are defined in the [glossary](../GLOSSARY.md); the safety design is [ADR 0009](../docs/adr/0009-safety-api-hardware-protection.md).

Application-specific tests (for example the uSolarVerter's L0/L1/L2 suites) live on their application branches; this folder holds what every board shares.

## Run

```bash
tests/run_all.sh --l0 --target   # every L0 test, including the on-target ones: REFLASHES the Spin over the ST-Link
tests/run_all.sh --l0            # L0 without hardware: on-target tests report BLOCKED
bash tests/l0/safety_supervision/run.sh --run   # one on-target test as a campaign of its own
bash tests/common/target_preflight.sh           # what an on-target test needs, and how to fix what is missing
```

Each campaign writes `tests/out/<date>_<sha>_<time>_<id>/` with one log per test script, `record.md` (verdicts), `report.html` (the review page: purpose and conditions, snapshot, results, figures per test case) and updates the control center.

**On-target tests** run on a bare Spin (no shield, no power stage) through an ST-Link. They reflash the board: flash env USB again before a bench session.

## Layout

| Path | Content |
|---|---|
| [`run_all.sh`](run_all.sh) | Campaign runner: every `tests/l0/*/run.sh`, then record, review page, control center |
| [`common/`](common/README.md) | Check helper, build helper, record writer (`make_record.py`), review page (`review_report.py`), control center, catalogue checker, on-target preflight |
| [`firmware/`](firmware/) | Test firmware environments (`pio_tests.ini`) and their apps; never for a power board on a bench |
| [`l0/safety_supervision/`](l0/safety_supervision/README.md) | L0-SUP: CPU supervision on the Spin (watchdog, fatal-error handler, HRTIM system fault), issue luizvilla/Core#16 |
| `catalogue.json` | Every test, defined once |
| `out/` | Campaigns (git-ignored) and `control-center.html` |

## Adding a test

1. Add it to `catalogue.json` (tab, criterion, conditions, source, status) and run `python3 tests/common/catalogue.py`.
2. Write it in `tests/l0/<name>/` with a `run.sh` and a `README.md`; add its object code to `object_code()` in `run_all.sh`. Each criterion prints one result line: `@@<TAB>ID<TAB>PASS|FAIL|INFO|BLOCKED<TAB>what<TAB>expected<TAB>measured`.
3. **Results reach the control center only from a campaign under `tests/out/`**: `run_all.sh`, or a `run.sh` that writes a campaign of its own. Never report results into a scratch folder.
4. A test whose result needs its conditions, a snapshot or a figure writes a **test-case folder** the review page recognises, rendered with the four parts. On-target tests write `target_manifest.json` (title, purpose, criteria prefixes, conditions, target snapshot, report file, figure kind); a new figure kind gets its branch in `common/review_report.py`, using the page's palette.
5. **Done means visible**: run it through a campaign, open `tests/out/control-center.html`, check the test shows its newest result and its link opens the rendered report.
