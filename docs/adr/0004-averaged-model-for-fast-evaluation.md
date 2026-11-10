---
status: proposed
date: 2026-11-10
---

# Averaged models for fast evaluation of ideas; validation only on the switched model

[ADR 0002](0002-switched-plant-only.md) keeps a single plant model for L1, the switched Pulsim circuit. On Luiz Villa's laptop a switched run of the inverter bench sequence (2 s at a 50 ns step) takes about 70 minutes, so each idea costs more than an hour. Ruling out ideas costs less in an averaged model, which runs in minutes. This amends ADR 0002; it does not replace it.

## Decision

- An averaged (or analytic) model may be used to evaluate an idea quickly before committing switched-model time to it: tuning candidates, a fidelity ladder that adds plant elements one at a time, a proposed fix.
- **Validation comes only from the switched model.** An averaged-model result is never a verdict on the firmware. It can only decide whether an idea is worth a switched run.
- **An averaged model is used only when the user asks for it**, never on the AI assistant's initiative. The user then always knows which kind of simulation runs.
- Averaged-model tests are design tests, at a level of their own, **D** (design), with IDs `D-<object>-<nn>` (for example `D-GFL-01`). They are filed in the catalogue like other tests; their results are INFO and never PASS or FAIL against a requirement.

## Considered options

- **Switched model only (ADR 0002 unchanged):** every idea costs about 70 minutes on the slower machine, and a failure in the full model does not say which of its elements caused it.
- **Averaged model as a second verdict model** (the option ADR 0002 rejected): still rejected. Disagreements between two verdict models become questions about the models.
- **Averaged model for fast evaluation only, at the user's request:** chosen.

## Consequences

- Disagreements between an averaged result and the switched model are expected; they are information about which plant element matters, not a defect to reconcile.
- The level table of the test procedure (§2) gets a D row.
- To agree with Jean Alinei, since ADR 0002 was taken at his request.
