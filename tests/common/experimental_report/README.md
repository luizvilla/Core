# Experimental report export (prototype)

Goal: one self-contained HTML page per bench session or evening of work, to send to someone who was not there (a colleague, or an AI assistant helping them). It summarizes what was tried and in what order, shows the evidence, and says how to reproduce. The campaign review pages (`tests/common/review_report.py`) judge one campaign against its criteria. This page tells the story across several campaigns.

## Current state

| File | What |
|---|---|
| `recap_2026-11-13_evening.py` | First instance, written by hand: the 2026-11-13 evening (L2-INV-09 to L2-INV-13, the bridge incident, the L0-TIM-03 stack-overflow freeze). Output: `docs/testing/recaps/2026-11-13_evening_recap.html` (about 6 MB, works offline) |

```bash
python3 tests/common/experimental_report/recap_2026-11-13_evening.py [output.html]
```

It needs the campaigns under `tests/out/2026-11-13_*`. These are local and not in git, so the script only runs on the machine that holds them.

## What the page holds (the layout to keep)

1. A header with the branch, the commit range and the sources (worklog, campaigns).
2. A short summary of three lines.
3. **A table of steps in order**: time, step, commit, hypothesis, finding and outcome badge (rejected / confirmed / new defect / reproduced / root cause).
4. Evidence per step:
   - interactive uPlot figures, with data embedded. Scope traces are averaged over one switching period for the low-frequency view, and a full-rate zoom shows the ripple.
   - spectra and replays recomputed from `waveforms.npz`, and the firmware scope dumps (`firmware_scope/*.csv`).
   - telemetry parsed from `console.txt`.
   - MSO54 screenshots embedded in base64.
5. Incident and console excerpts, quoted verbatim.
6. Reproduction: hardware, tools, the exact commands and the expected symptom.
7. Where the files are.

## What is hard-coded now, to turn into inputs

- The steps table and the narrative, taken from `docs/testing/worklog.md`. A generator could read the worklog entries in a time window and the `record.md` / `bench_manifest.json` of each campaign.
- The choice of captures and screenshots to show. A generator could use the ones listed in each `bench_manifest.json`, with the operator's labels.
- Channel meanings, which `bench_session.json` already records.
- The figure helper `fig()` and the theme. These should merge with `tests/common/interactive_plot.py`, so that the review pages and this export share one figure code and the same conventions (procedure §8.5).
- The firmware scope dump rate (2 kHz assumed here). The console prints it with every dump (`scope: 256 samples, … Hz`), so it can be read from there.
