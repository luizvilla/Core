# Firmware block and L1-FWB-01

The **firmware block** is the application's `controlStep()` with the boost, grid observer and grid-following inverter, compiled unchanged into a shared library that Pulsim runs as a C-block every 100 µs. Catalogue: [procedure §6.6](../../../docs/testing/test-procedure.md#66-l1-fwb--firmware-block-infrastructure).

## Files

| File | Role |
|---|---|
| [`firmware_block.cpp`](firmware_block.cpp) | C-block ABI (`pulsim_cblock_init/step/term`) around `controlInit()` / `controlStep()`; wire order of contract v1 |
| [`build.sh`](build.sh) | Builds the `.so` from the same sources and include paths as L0 (`-Werror` on firmware code, `-fPIC -O2`, no sanitizer) |
| [`contract.py`](contract.py) | Wire names in order, `add_firmware_block()`, `add_command_timeline()`, `output_node()` |
| [`test_firmware_block.py`](test_firmware_block.py) | L1-FWB-01 against a stand-in circuit |

## Build and run

```bash
bash tests/l1/firmware_block/build.sh /tmp/firmware_block.so
.venv/bin/python -I tests/l1/firmware_block/test_firmware_block.py /tmp/firmware_block.so /tmp/L1-FWB-01.csv
```

`SINGLE_PHASE_INVERTER_DIR` selects another library checkout, as for L0.

## Using the block in a circuit

```python
import pulsim, contract
b = pulsim.CircuitBuilder()
# ... plant model: every component first ...
contract.add_command_timeline(b, "cmd_mode", 0.0, [(0.2, 2.0)])   # IDLE, BUSREG from 0.2 s
contract.add_command_timeline(b, "cmd_sync", 1.0)
contract.add_command_timeline(b, "cmd_vref", 400.0)
handle = contract.add_firmware_block(b, "firmware_block.so", {
    "VLow": ("v", "pv"), "ILow1": ("i", "L1"), ...,                  # every name in contract.INPUTS
    "mode": ("v", "cmd_mode"), "sync_request": ("v", "cmd_sync"), "v_bus_ref": ("v", "cmd_vref"),
}, sim_dt=2e-5)                                                      # last: no component after this
res = pulsim.simulate(b, t_end=1.0, dt=2e-5)
duty = res.v(contract.output_node("duty_leg1"))
```

Each output is a voltage to `gnd` equal to the value (duty 0–1, enables 0/1, angle in rad, …).

## What the block adds to `controlStep()`

Only what the critical task in `main.cpp` does around it, plus one simulator adaptation:

| Adaptation | Why |
|---|---|
| `en_leg1/2` = `boost_pwm_update && leg_enable` | `main.cpp` stops the legs on a mode change and updates them only outside IDLE |
| `mode` rounded and clamped to 0–3; `sync_request`, `vac_valid` > 0.5 = true | Commands and flags travel as analog levels |
| The call at t = 0 is skipped (outputs 0) | Pulsim calls the block before solving the circuit, with all inputs at 0. Boot with a missing first VAC sample is tested at L0 (L0-APP-03.c) |
| Wire counts checked; `status` < 0 on mismatch or `controlInit` failure | A wiring error must be visible, not silent |

## L1-FWB-01 stand-in circuit

Fixed sources for every input (30 V PV, 390 V bus, zero currents, 230 V / 50 Hz grid, VAC valid), a command timeline (IDLE, then BUSREG at 0.2 s, sync requested, 400 V reference), and an RC low-pass (1 kΩ, 10 µF) on `duty_leg1`. 1.5 s simulated at a 20 µs step.

| Criterion | Checks |
|---|---|
| .a | Contract versions match; `status` is 0 at every step |
| .b | 15 001 block calls over 1.5 s; outputs constant inside each 100 µs period |
| .c | Observer reads 230 V ± 1 % and 50 ± 0.05 Hz: VAC arrives in volts |
| .d | One mode change, within one period of 0.2 s; legs off in IDLE, leg 1 on in BUSREG |
| .e | RC-filtered duty equals `duty_leg1`: outputs drive circuit elements |
| .f | Bridge PWM and relay never requested |
| .g | Inverter synchronized at 1.5 s |
| .j | No Pulsim divergence warning; every output finite |
| .h, .i | INFO: time to synchronization (compare L0-APP-05.c), wall time per simulated second |

## History

Added in commit 7 of the plan. Contract v1 (v0 plus `sync`, `mode_changed`, `status`).
