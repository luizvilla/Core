# Simulink integration plan

Design plan for driving the Twist board from an actual Simulink model instead of
`comm_script.m`'s manual reference-ramp loop, so a real Simulink controller can close the loop
against live `V1`/`V2` measurements. **Status: plan only** — none of the files below exist yet;
this document is the reference a follow-up implementation task executes against, the same way
[`README.md`](README.md) served as the design plan before `ShieldDevice.m` etc. were built.

## Why split into three pieces

A single monolithic block (open connection, send commands, read measurements, all in one) would
work, but doesn't map naturally onto how a Simulink block diagram is normally read: separate
send/receive signals, discovery happening once regardless of diagram layout. The three pieces
below match that expectation and let the two runtime blocks be placed wherever they're needed in
a real controller diagram.

## Architecture

**Three pieces**, all to live in `src/matlab/`:

1. **`getShieldConnection.m`** (+ **`releaseShieldConnection.m`**) — a shared connection
   singleton holding a `ShieldDevice` handle. First call from *either* downstream block does:
   `findShieldDevicePort` → `ShieldDevice` construction → the one-time setup sequence
   (`IDLE`→`BUCK LEG1/LEG2 ON`→`LEG LEG1/LEG2 ON`→`REFERENCE LEG1 V1 5`→`POWER_ON`, exactly
   `comm_script.m`'s existing setup block). Subsequent calls return the same handle. This avoids
   relying on Simulink block execution order (not guaranteed between unconnected blocks) or on
   editing `InitFcn`/`StopFcn` callbacks embedded in a binary `.slx` (not code-reviewable) —
   everything stays in plain, diffable `.m` files. `releaseShieldConnection.m` mirrors this:
   first call sends `IDLE` and clears the stored handle (mirrors `comm_script.m`'s
   `onCleanup`/`safeIdle`); later calls are no-ops.
   - **Implementation detail found while building this**: MATLAB `persistent` variables are
     scoped per-function, not shared across separate `.m` files — so `getShieldConnection.m`
     and `releaseShieldConnection.m`, as two independent top-level functions, can't directly
     share a `persistent` handle with each other the way the "plain function holding a
     `persistent` handle" description above implies in isolation. A third file,
     **`shieldConnectionSingleton.m`**, holds the actual `persistent` state and exposes
     `get`/`set`/`clear` actions; both public functions delegate to it. This was chosen over
     `global` (harder to review, namespace risk) or storing the handle in the base workspace via
     `assignin`/`evalin` (pollutes the user's interactive workspace, vulnerable to a stray
     `clear` in the command window). See Step 1's "Verified" note for how this was confirmed.
   - Testability hook (mirroring `findShieldDevicePort`'s `Interactive` flag and
     `comm_script.m`'s `EnablePlot` flag): accept optional `VendorID`/`ProductID`/`ForcedPort`
     arguments so a no-hardware test can inject the pty fake-board's port instead of relying on
     real USB auto-discovery. `Interactive` defaults to `false` here (unlike
     `findShieldDevicePort`'s own default of `true`), since `getShieldConnection` is meant to
     run unattended from a Simulink block rather than from a human at a terminal.

2. **`ShieldSendBlock.m`** — a `matlab.System` object. Inputs: `Ref1` (→ `REFERENCE LEG1 V1`),
   `Ref2` (→ `REFERENCE LEG2 V2`) — matching `comm_script.m`'s existing two-leg demo exactly
   rather than a more generic/configurable command block (avoid speculative generality; extend
   later if a different demo needs it). `setupImpl` calls `getShieldConnection()`; `stepImpl`
   sends both `REFERENCE` commands; `releaseImpl` calls `releaseShieldConnection()`.

3. **`ShieldGetBlock.m`** — a `matlab.System` object. No inputs; outputs `V1`, `V2`. Same
   `setupImpl`/`releaseImpl` pattern; `stepImpl` calls `getMeasurement('V1')`/
   `getMeasurement('V2')`.

### Simulink-specific implementation details (easy to get wrong — call out up front)

- Both System objects must force **interpreted execution**, not code generation
  (`getSimulateUsingImpl` returning `'Interpreted execution'`, non-tunable) — `serialport` and
  the persistent-handle singleton pattern are not code-generation-compatible.
- Both must declare a **discrete sample time** via `getSampleTimeImpl` (mask parameter, e.g.
  `SampleTime`, no baked-in default) — real round-trip time per `REFERENCE`+`getMeasurement`
  exchange was measured at ~1 s/frame in `comm_script.m`'s Step 4 real-hardware verification (see
  [`README.md`](README.md)), so the model's step rate must be set accordingly, not left
  continuous/default.

## Support files needed

| File | Purpose |
|---|---|
| `getShieldConnection.m` | Shared connection accessor (see above). |
| `releaseShieldConnection.m` | Matching teardown accessor — sends `IDLE`, clears the handle, no-op on repeat calls. |
| `shieldConnectionSingleton.m` | Private state holder (`get`/`set`/`clear`) shared by the two functions above — needed because MATLAB `persistent` variables can't be shared across separate `.m` files (see "Implementation detail" note above). Not called directly by anything else. |
| `ShieldSendBlock.m` | `matlab.System` — sends `REFERENCE` to both legs each step. |
| `ShieldGetBlock.m` | `matlab.System` — reads `V1`/`V2` each step. |
| `build_shield_test_model.m` | Programmatic builder for the test model (below) — not hand-drawn, so construction is code-reviewable. |
| `shield_test_model.slx` | Generated by the script above; the integration test harness. |

## Testing strategy

Two layers, mirroring [`README.md`](README.md)'s "No-hardware validation procedure":

1. **No-hardware unit test** — reuse the existing `fake_board.py` pty-loopback harness
   unchanged. Call the System objects' public `step()`/`release()` API directly (no `.slx` model
   needed — this is exactly what `matlab.System` objects are for) against the fake port,
   verifying command formatting/pacing/parsing the same way Steps 1–2 verified `ShieldDevice`/
   `findShieldDevicePort`.
2. **Simulink-diagram integration test against a real, if simple, model** — not just ad hoc block
   wiring; a genuine small demo model that exercises all three pieces together end to end (see
   "Simple Simulink test model" below), run headlessly via `sim()` under `matlab -batch`. First
   against the pty fake board (catches Simulink-plumbing bugs — sample time, mask parameters,
   port wiring — that direct unit testing wouldn't), then, gated by an explicit real-hardware
   safety confirmation (the same pattern used throughout this project — see the `README.md` Step
   3/4 notes on not sending power commands to real hardware without confirming the bench is
   safely configured first), against the real powered board.

## Simple Simulink test model

A `.slx` model that wires the three pieces together into something equivalent to
`comm_script.m`'s demo, but as an actual block diagram — this is both the integration test
harness and a usable starting point for a real controller later. Built **programmatically**, not
hand-drawn, via `build_shield_test_model.m` (`new_system`/`add_block`/`add_line`/`set_param`/
`save_system`) so the model is regeneratable and its construction is code-reviewable, consistent
with everything else in this project being diffable `.m` source rather than opaque binary state.

**Contents of `shield_test_model.slx`**:

- A reference-source block reproducing `comm_script.m`'s triangular ramp (start 5, step +0.5,
  wrap at 15) — a `Repeating Sequence Stair` block (or a small `MATLAB Function` block if that's
  easier for the exact wrap behavior) feeding the *same* signal into both `ShieldSendBlock`
  inputs (`Ref1` and `Ref2`), matching `comm_script.m` driving both legs off one ramp.
- One `ShieldSendBlock` instance, one `ShieldGetBlock` instance — this is the actual thing under
  test, confirming the two blocks correctly share one connection via `getShieldConnection`.
- `Scope` blocks on `V1`/`V2` — the block-diagram equivalent of `comm_script.m`'s live plot, for
  visual/interactive confirmation the same way the live plot itself was visually confirmed.
- `To Workspace` blocks logging `V1`/`V2` (and the reference signal) to the base workspace, so
  the no-hardware and real-hardware test passes can assert on logged data programmatically
  instead of only eyeballing the Scope — mirrors how `test_connection.m`/`comm_script.m`'s
  headless runs printed pass/fail instead of relying on a human watching.
- Model configuration: discrete fixed-step solver, step size matching the blocks' `SampleTime`
  mask parameter (~1 s, per the measured real round-trip time above).
- Stop time bounded and settable (short vs. long run), so the no-hardware pass and an initial
  real-hardware sanity pass can both run briefly (a handful of steps) before any longer real run
  — mirrors how Step 4's real-hardware verification started headless/bounded (`MaxCycles`)
  before anything longer.

**Testing this specific model** (its own explicit two-layer pass, not just folded into the
System-object unit tests above):

- *No-hardware*: run `shield_test_model` via `sim()` against the pty fake board (the connection
  singleton's `ForcedPort` override, passed through as a model/block parameter), for a short
  bounded stop time. Assert the `To Workspace`-logged `V1`/`V2` match the fake board's planted
  values, and that the model runs to completion without error — proves the two blocks correctly
  share a single `getShieldConnection()` instance inside an actual compiled Simulink model, not
  just when called directly from a script.
- *Real hardware*: same model, same assertions in spirit, run against the real powered board —
  gated by an explicit confirmation step first, starting with a short bounded run before a longer
  one. Confirms `V1` tracks the triangular reference through actual Simulink-scheduled steps, the
  way `comm_script.m`'s powered run confirmed it through a plain MATLAB loop.

## Commit sequence

One commit per file/milestone, mirroring how `ShieldDevice.m` etc. landed (see `git log --oneline
-- src/matlab/`):

1. `getShieldConnection.m` + `releaseShieldConnection.m` — `feat(matlab): add shared shield connection singleton`
2. `ShieldSendBlock.m` — `feat(matlab): add Simulink block for sending shield reference commands`
3. `ShieldGetBlock.m` — `feat(matlab): add Simulink block for reading shield measurements`
4. `build_shield_test_model.m` (+ generated `shield_test_model.slx`) — `test(matlab): add Simulink test model wiring the shield blocks together`, including its no-hardware pty pass
5. Real-hardware verification of `shield_test_model.slx` (short bounded run, then a longer one,
   gated by explicit confirmation) recorded into this document —
   `docs(matlab): record Simulink block verification results`

## Work sequence (resumable, one block per commit)

Same resumable pattern as `README.md`: precondition, resume check via `git log`, a `Do`
checklist, a definition of done, and the commit to make once done. To resume after a break, run
`git log --oneline -- src/matlab/` and find the highest-numbered step whose commit already
exists.

### Step 1 — `getShieldConnection.m` + `releaseShieldConnection.m`

- **Precondition**: none (first step).
- **Resume check**: `git log --oneline -- src/matlab/getShieldConnection.m`.
- **Do**:
  - [x] Implement `getShieldConnection` with a shared `ShieldDevice` handle (via
        `shieldConnectionSingleton.m` — see the "Implementation detail" note in Architecture
        above); first call discovers the port (`findShieldDevicePort`, with
        `VendorID`/`ProductID`/`ForcedPort` pass-through options), constructs `ShieldDevice`,
        and runs the one-time setup sequence.
  - [x] Implement `releaseShieldConnection`: first call sends `IDLE` and clears the stored
        handle; subsequent calls are no-ops.
  - [x] `checkcode` clean on all three files (`getShieldConnection.m`,
        `releaseShieldConnection.m`, `shieldConnectionSingleton.m`).
- **Definition of done**: calling `getShieldConnection()` twice in a row (e.g. against a pty
  loopback) returns the *same* handle both times, and only one discovery/setup sequence is
  observed on the wire; `releaseShieldConnection()` called twice sends exactly one `IDLE`.
  - **Verified 2026-07-13** without real hardware, using the same Python `pty`-based virtual
    serial loopback pattern established in `README.md`'s Step 1: `checkcode` reported no issues
    on all three files. Wire trace confirmed exactly the expected 7-command setup sequence
    (`IDLE`, `BUCK LEG1 ON`, `BUCK LEG2 ON`, `LEG LEG1 ON`, `LEG LEG2 ON`,
    `REFERENCE LEG1 V1 5.00000`, `POWER_ON`) on the first `getShieldConnection()` call; a
    second call returned the identical handle (MATLAB handle `==` comparison) with **zero**
    additional wire traffic; `releaseShieldConnection()` sent exactly one `IDLE`, and a second
    call was a true no-op (no additional traffic); a further `getShieldConnection()` call after
    release correctly performed a fresh 7-command setup and returned a *new*, distinct handle —
    confirming the full get/release lifecycle, not just the two behaviors named in the
    definition of done above. 15 total commands logged across the whole sequence
    (7 + 0 + 1 + 0 + 7), exactly as expected.
  - **Discovery re-confirmed against real hardware 2026-07-13**: with a board attached, a
    sysfs check (`idVendor`/`idProduct` under `/sys/class/tty/ttyACM0`) showed VID `2fe3`/PID
    `0101`, and `findShieldDevicePort()` run directly in MATLAB against it returned
    `/dev/ttyACM0` on the first try — no fallback needed this time, unlike the PID
    `0x0100`/`0x0101` variability noted in `README.md`'s Step 2. **Scope note**: this only
    exercises `findShieldDevicePort` (discovery), not `getShieldConnection` as a whole —
    opening a connection would run the full setup sequence and start power flow, which needs
    the same explicit go-ahead as the rest of this project's real-hardware steps. A full
    real-hardware pass of `getShieldConnection`/`releaseShieldConnection` (open, setup, park)
    is still pending — see Step 5.
- **Commit**: `feat(matlab): add shared shield connection singleton`

### Step 2 — `ShieldSendBlock.m`

- **Precondition**: Step 1 committed.
- **Resume check**: `git log --oneline -- src/matlab/ShieldSendBlock.m`.
- **Do**:
  - [ ] `classdef ShieldSendBlock < matlab.System` with `Ref1`/`Ref2` inputs.
  - [ ] `setupImpl` calls `getShieldConnection()`; `stepImpl` sends both `REFERENCE` commands;
        `releaseImpl` calls `releaseShieldConnection()`.
  - [ ] Force interpreted execution (`getSimulateUsingImpl`).
  - [ ] Declare discrete sample time via `getSampleTimeImpl` with a `SampleTime` mask parameter.
  - [ ] `checkcode` clean.
- **Definition of done**: directly calling `step(obj, ref1, ref2)` against a pty loopback
  produces the expected `REFERENCE LEG1 V1 <ref1>` / `REFERENCE LEG2 V2 <ref2>` wire traffic.
- **Commit**: `feat(matlab): add Simulink block for sending shield reference commands`

### Step 3 — `ShieldGetBlock.m`

- **Precondition**: Step 1 committed (can be built in parallel with Step 2; both only depend on
  Step 1).
- **Resume check**: `git log --oneline -- src/matlab/ShieldGetBlock.m`.
- **Do**:
  - [ ] `classdef ShieldGetBlock < matlab.System` with `V1`/`V2` outputs, no inputs.
  - [ ] `setupImpl`/`releaseImpl` matching Step 2's pattern; `stepImpl` calls
        `getMeasurement('V1')`/`getMeasurement('V2')`.
  - [ ] Force interpreted execution; declare discrete sample time.
  - [ ] `checkcode` clean.
- **Definition of done**: directly calling `step(obj)` against a pty loopback returns the exact
  planted `V1`/`V2` values from a synthetic telemetry frame.
- **Commit**: `feat(matlab): add Simulink block for reading shield measurements`

### Step 4 — `build_shield_test_model.m` + `shield_test_model.slx`

- **Precondition**: Steps 1–3 committed.
- **Resume check**: `git log --oneline -- src/matlab/build_shield_test_model.m`.
- **Do**:
  - [ ] Write `build_shield_test_model.m` per "Simple Simulink test model" above: reference
        source → both `ShieldSendBlock` inputs; `ShieldGetBlock` outputs → `Scope` +
        `To Workspace`; discrete fixed-step config; bounded/settable stop time.
  - [ ] Run the builder to generate `shield_test_model.slx`; commit both.
  - [ ] No-hardware pass: run the model against the pty fake board for a short bounded stop
        time; assert logged `V1`/`V2` match the planted values.
- **Definition of done**: the no-hardware pass above completes with no errors and correct logged
  values, proving both blocks share one connection inside an actual compiled model.
- **Commit**: `test(matlab): add Simulink test model wiring the shield blocks together`

### Step 5 — Real-hardware verification

- **Precondition**: Step 4 committed and passing its no-hardware pass.
- **Resume check**: `git log --oneline -- src/matlab/simulink_blocks.md` (look for a commit
  whose message matches this step, or check this file's own "Verified" notes once added).
- **Do**:
  - [ ] Confirm with the user before sending any power-enabling commands to real hardware (same
        pattern as `README.md`'s Step 3/4 — do not assume prior authorization carries over).
  - [ ] Run `shield_test_model` against the real board for a short bounded stop time first.
  - [ ] Run a longer pass and visually confirm the Scope tracks the triangular reference.
  - [ ] Record results (timings, logged values, any discrepancies) into this document, the same
        way `README.md` records dated "Verified" notes per step.
- **Definition of done**: a real-hardware run completes with `V1` tracking the reference and the
  board returns to `IDLE` afterward, with results documented here.
- **Commit**: `docs(matlab): record Simulink block verification results`

## Next steps

- [x] **Step 1** — `getShieldConnection.m`/`releaseShieldConnection.m` implemented,
  `checkcode`-clean, and verified against a pty loopback (see Step 1's "Verified" note). Picked
  up one implementation detail not anticipated in the original architecture description: a
  third file, `shieldConnectionSingleton.m`, was needed to actually share state between the two
  public functions (documented in Architecture and the Support files table above).
- [ ] **Step 2** and **Step 3** (`ShieldSendBlock.m`, `ShieldGetBlock.m`) can proceed in
  parallel — both only depend on Step 1, not on each other. **This is the next action.**
- [ ] **Step 4** — `build_shield_test_model.m` + `shield_test_model.slx`, plus its no-hardware
  pass.
- [ ] **Step 5** — real-hardware verification, gated by explicit confirmation.

If resuming cold: run `git log --oneline -- src/matlab/` to see which of the files above already
have commits, and continue from the first unchecked item.
