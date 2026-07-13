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
  - [x] `classdef ShieldSendBlock < matlab.System` with `Ref1`/`Ref2` inputs (named via
        `getInputNamesImpl`).
  - [x] `setupImpl` calls `getShieldConnection()`; `stepImpl` sends both `REFERENCE` commands;
        `releaseImpl` calls `releaseShieldConnection()`.
  - [x] Force interpreted execution (`getSimulateUsingImpl` + `showSimulateUsingImpl`, both
        `Static Access=protected` — see implementation note below).
  - [x] Declare discrete sample time via `getSampleTimeImpl` with a `SampleTime` mask parameter
        (via `obj.createSampleTime('Type','Discrete','SampleTime',obj.SampleTime)`).
  - [x] `checkcode` clean.
  - **Implementation detail found while building this, not anticipated in the architecture
    description**: `matlab.System` does not automatically support `MyClass('Prop', val, ...)`
    construction — an explicit constructor calling `setProperties(obj, nargin, varargin{:})`
    is required, or construction fails with "No matching constructor found for superclass
    'matlab.system.SystemInterface'". Confirmed against real MathWorks source
    (`toolbox/shared/seriallib_blocks/+system/SerialReceive.m`, a real serial-hardware System
    object shipped with Instrument Control Toolbox) before implementing, rather than relying on
    memory for the exact `matlab.System` override method signatures — that same source also
    confirmed the `getSampleTimeImpl`/`createSampleTime` pattern, and a separate real example
    (`+codertarget/+armM4/+blocks/TcpSend.m`) confirmed `getSimulateUsingImpl`/
    `showSimulateUsingImpl` must be `methods (Static, Access = protected)`, not plain instance
    methods.
- **Definition of done**: directly calling `step(obj, ref1, ref2)` against a pty loopback
  produces the expected `REFERENCE LEG1 V1 <ref1>` / `REFERENCE LEG2 V2 <ref2>` wire traffic.
  - **Verified 2026-07-13** without real hardware, using the same pty-loopback pattern as Step
    1: `checkcode` reported no issues. `ShieldSendBlock('ForcedPort', port)` constructed
    correctly; `step(obj, 7.25, 3.5)` produced, on the wire, the 7-command
    `getShieldConnection` setup sequence (first call) followed by exactly
    `s_LEG1_r_V1_7.25000` and `s_LEG2_r_V2_3.50000` — the correctly-formatted `REFERENCE`
    commands for the given `ref1`/`ref2` values; `release(obj)` then sent exactly one `d_i`
    (`IDLE`) via `releaseShieldConnection`. Full setup→step→release lifecycle confirmed
    end-to-end.
- **Commit**: `feat(matlab): add Simulink block for sending shield reference commands`

### Step 3 — `ShieldGetBlock.m`

- **Precondition**: Step 1 committed (can be built in parallel with Step 2; both only depend on
  Step 1).
- **Resume check**: `git log --oneline -- src/matlab/ShieldGetBlock.m`.
- **Do**:
  - [x] `classdef ShieldGetBlock < matlab.System` with `V1`/`V2` outputs (named via
        `getOutputNamesImpl`), no inputs.
  - [x] `setupImpl`/`releaseImpl` matching Step 2's pattern; `stepImpl` calls
        `getMeasurement('V1')`/`getMeasurement('V2')`.
  - [x] Force interpreted execution; declare discrete sample time — identical pattern to
        `ShieldSendBlock.m` (constructor via `setProperties`, `createSampleTime`, static
        `getSimulateUsingImpl`/`showSimulateUsingImpl`), reused directly since Step 2 already
        confirmed these against real MathWorks source.
  - [x] `checkcode` clean.
- **Definition of done**: directly calling `step(obj)` against a pty loopback returns the exact
  planted `V1`/`V2` values from a synthetic telemetry frame.
  - **Verified 2026-07-13** without real hardware: `checkcode` reported no issues.
    `ShieldGetBlock('ForcedPort', port)` constructed correctly; `setupImpl` ran the same
    7-command `getShieldConnection` setup sequence; `step(obj)` correctly returned
    `[8.11111, 2.22222]`, the exact `V1`/`V2` values planted in a synthetic 16-field telemetry
    frame streamed by the fake board (with an interleaved non-16-field debug line, confirming
    the field-count filter still works when called through the block); `release(obj)` sent
    exactly one `IDLE`. Full setup→step→release lifecycle confirmed, matching Step 2's result.
  - **Amended during Step 4**: placing `ShieldGetBlock` inside an actual Simulink model (not
    just calling `step()` directly) surfaced an error this direct-call test couldn't catch —
    Simulink's automatic output-property inference tries to statically analyze `stepImpl` via
    code generation *regardless of the "Simulate using" setting*, and fails because
    `getShieldConnection` uses `inputParser` (not codegen-compatible):
    `Function inputParser is not supported for code generation`. Fixed by explicitly
    implementing `getOutputSizeImpl`/`getOutputDataTypeImpl`/`isOutputComplexImpl`/
    `isOutputFixedSizeImpl` (declaring both outputs as `[1 1]` `double`, real, fixed-size) so
    Simulink never attempts that inference — the same four methods present in the real
    `SerialReceive.m` example checked during Step 2, which should have been included from the
    start. Re-verified: the direct-`step()` pty-loopback test above still passes unchanged.
- **Commit**: `feat(matlab): add Simulink block for reading shield measurements`,
  amended by `fix(matlab): declare ShieldGetBlock output properties for Simulink inference`

### Step 4 — `build_shield_test_model.m` + `shield_test_model.slx`

- **Precondition**: Steps 1–3 committed.
- **Resume check**: `git log --oneline -- src/matlab/build_shield_test_model.m`.
- **Do**:
  - [x] Write `build_shield_test_model.m` per "Simple Simulink test model" above: reference
        source (`Repeating Sequence Stair`, `OutValues = [5.5:0.5:14.5, 5.0]`, reproducing
        `comm_script.m`'s ramp exactly) → both `ShieldSendBlock` inputs; `ShieldGetBlock`
        outputs → one 2-port `Scope` + separate `V1`/`V2` `To Workspace` blocks; ramp also
        logged to its own `To Workspace`; discrete fixed-step solver (`FixedStepDiscrete`,
        step = 1s matching the blocks' `SampleTime`); `StopTime` defaults to 5s (bounded, and
        overridable via `set_param` before `sim()` for a longer real run in Step 5).
  - [x] Run the builder to generate `shield_test_model.slx`; commit both.
  - [x] No-hardware pass: run the model against the pty fake board for a short bounded stop
        time; assert logged `V1`/`V2` match the planted values.
- **Definition of done**: the no-hardware pass above completes with no errors and correct logged
  values, proving both blocks share one connection inside an actual compiled model.
  - **Implementation details confirmed against real Simulink** (not assumed) before writing the
    final builder: the "MATLAB System" block's library path
    (`simulink/User-Defined Functions/MATLAB System`) and its class-selection parameter (`System`
    — not `SystemObjectClassName`, which doesn't exist); that setting `System` auto-exposes the
    class's `Nontunable` properties as directly `set_param`-able block parameters; `Repeating
    Sequence Stair`'s `tsamp` is a scalar sample time, not a per-value timestamp vector as first
    assumed; and that `set_param(model, 'ReturnWorkspaceOutputs', 'on')` plus
    `simOut = sim(model); get(simOut, 'VarName')` retrieves `To Workspace` logs without touching
    the base workspace (consistent with this project's earlier preference — see
    `getShieldConnection`'s design note — for avoiding base-workspace pollution).
  - **Verified 2026-07-13** without real hardware, using the same pty fake-board pattern as
    Steps 1–3 (continuously-streamed synthetic telemetry, `V1 = 9.87654`, `V2 = 1.23456`),
    `ForcedPort` set via `set_param` on both blocks before `sim()`: the model ran to completion
    (`StopTime = 5`, 6 sample points including `t=0`) with no errors.
    `V1_log`/`V2_log` were `[9.87654 9.87654 9.87654 9.87654 9.87654 9.87654]` and
    `[1.23456 1.23456 1.23456 1.23456 1.23456 1.23456]` — an exact match at every sample, every
    time. `Ref_log` was `[5.5 6 6.5 7 7.5 8]`, the correct start of the triangular ramp. Total
    wall-clock time ≈97s (model compile/load overhead plus ~1s/frame real-I/O-paced steps,
    consistent with the timing documented in `README.md`'s Step 4). This is the first
    confirmation that `ShieldSendBlock` and `ShieldGetBlock` correctly share one
    `getShieldConnection()` instance *inside a compiled Simulink model*, not just when called
    directly from a script (Steps 2–3's tests).
  - See Step 3's "Amended during Step 4" note for a `ShieldGetBlock.m` fix this pass required.
- **Commit**: `test(matlab): add Simulink test model wiring the shield blocks together`

### Step 5 — Real-hardware verification

- **Precondition**: Step 4 committed and passing its no-hardware pass.
- **Resume check**: `git log --oneline -- src/matlab/simulink_blocks.md` (look for a commit
  whose message matches this step, or check this file's own "Verified" notes once added).
- **Do**:
  - [x] Confirm with the user before sending any power-enabling commands to real hardware (same
        pattern as `README.md`'s Step 3/4 — do not assume prior authorization carries over).
  - [x] Run `shield_test_model` against the real board.
  - [x] Visually confirm the Scope tracks the triangular reference.
  - [x] Record results into this document.
- **Definition of done**: a real-hardware run completes with `V1` tracking the reference and the
  board returns to `IDLE` afterward, with results documented here.
  - **Verified 2026-07-13**: the user ran `shield_test_model.slx` directly (interactively, in a
    MATLAB desktop session) against the real, powered board and confirmed the `V1`/`V2`
    measurements tracked correctly on the Scope. This closes the loop the no-hardware pass in
    Step 4 couldn't reach — the fake board only ever produces fixed planted values, so this is
    the first confirmation that the actual analog control loop responds to a Simulink-scheduled
    reference the way it responded to `comm_script.m`'s plain-loop reference in `README.md`'s
    Step 4. Detailed logged values/timings weren't captured this run (interactive, visual
    confirmation only, run by the user rather than scripted); a scripted/logged real-hardware
    pass (mirroring the no-hardware pass's `V1_log`/`V2_log` assertions, run headless with a
    bounded `StopTime`) remains a reasonable follow-up if numeric confirmation is wanted later,
    but is not blocking — the plan's stated definition of done (tracking confirmed, board parked
    afterward) is met.
- **Commit**: `docs(matlab): record Simulink block verification results`

## Next steps

- [x] **Step 1** — `getShieldConnection.m`/`releaseShieldConnection.m` implemented,
  `checkcode`-clean, and verified against a pty loopback (see Step 1's "Verified" note). Picked
  up one implementation detail not anticipated in the original architecture description: a
  third file, `shieldConnectionSingleton.m`, was needed to actually share state between the two
  public functions (documented in Architecture and the Support files table above).
- [x] **Step 2** — `ShieldSendBlock.m` implemented, `checkcode`-clean, and verified against a
  pty loopback (see Step 2's "Verified" note): full setup→step→release lifecycle confirmed,
  correct `REFERENCE` formatting for both legs. Found that `matlab.System` needs an explicit
  constructor calling `setProperties` — confirmed against real MathWorks source before
  implementing rather than guessing from memory (see Step 2's implementation-detail note).
- [x] **Step 3** — `ShieldGetBlock.m` implemented, `checkcode`-clean, and verified against a
  pty loopback with a synthetic telemetry frame (see Step 3's "Verified" note): `step(obj)`
  correctly returned the exact planted `V1`/`V2` values, full setup→step→release lifecycle
  confirmed, matching Step 2's result and reusing its already-confirmed `matlab.System`
  patterns directly.
- [x] **Step 4** — `build_shield_test_model.m` + `shield_test_model.slx` implemented, and the
  no-hardware pass confirmed both blocks correctly share one connection inside a compiled
  model: `V1_log`/`V2_log` exactly matched the fake board's planted values at every one of 6
  sample points, `Ref_log` showed the correct triangular-ramp start. Required a fix to
  `ShieldGetBlock.m` (output-property declarations) that direct-`step()` unit testing in Step 3
  couldn't have caught — a real example of why the Simulink-diagram integration test layer
  exists on top of the unit-test layer, not instead of it.
- [x] **Step 5** — real-hardware verification. The user ran `shield_test_model.slx`
  interactively against the real powered board and confirmed `V1`/`V2` tracked the triangular
  reference correctly on the Scope (see Step 5's "Verified" note).

**All five steps complete.** `getShieldConnection`/`releaseShieldConnection`, `ShieldSendBlock`,
`ShieldGetBlock`, and `shield_test_model.slx` are implemented, statically clean, and verified at
every layer this plan called for: pty-loopback unit tests (Steps 1–3), a no-hardware Simulink
integration pass with logged-value assertions (Step 4), and a real-hardware interactive
confirmation (Step 5).

**Known scope limitation, not a defect**: `ShieldSendBlock`/`ShieldGetBlock` are hardcoded to
`REFERENCE LEG1 V1` / `REFERENCE LEG2 V2` and to reading `V1`/`V2` — matching `comm_script.m`'s
specific demo, not the full protocol surface `ShieldDevice` actually exposes (all `sendCommand`
actions — `LEG`, `CAPA`, `DRIVER`, `BUCK`, `BOOST`, `DUTY`, `CALIBRATE` — and all 16
measurement fields, not just `V1`/`V2`). Generalizing these blocks is tracked as separate
follow-up work, not part of this plan's original scope.

If resuming cold: run `git log --oneline -- src/matlab/` to see which of the files above already
have commits, and continue from the first unchecked item.
