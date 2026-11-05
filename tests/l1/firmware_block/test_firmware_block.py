## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file test_firmware_block.py
## @brief L1-FWB-01: the firmware block runs inside Pulsim against a stand-in circuit.
## @see README.md in this folder, docs/testing/test-procedure.md §6.7
"""L1-FWB-01: firmware block plumbing check.

Not a plant model. The stand-in circuit only provides fixed sources for every
contract input (30 V PV side, 390 V bus, zero currents, a 230 V / 50 Hz grid),
step sources for the operator commands (IDLE, then BUSREG at 0.2 s, with a
sync request), and an RC filter on one output. It checks that the compiled
firmware runs in Pulsim at the control period, receives its inputs in
physical units, receives the commands, and drives circuit elements with its
outputs. Closed-loop behaviour is L1-APP, with the real plant model.

Usage: test_firmware_block.py <firmware_block.so> [csv_out]
"""

from __future__ import annotations

import sys
import time
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from check import Criterion, finish, info  # noqa: E402
import contract  # noqa: E402

T_END = 1.5               ## Simulated time (s): observer qualification (1 s) + sync dwell.
SIM_DT = 2e-5             ## Simulation step (s): five steps per control period.
MODE_STEP_S = 0.2         ## IDLE → BUSREG at this time (s).
V_BUS = 390.0             ## Fixed bus voltage (V), below the 400 V reference.
V_PV = 30.0               ## Fixed PV-side voltage (V).
GRID_RMS = 230.0          ## Grid RMS voltage (V).
GRID_HZ = 50.0            ## Grid frequency (Hz).
RC_R, RC_C = 1e3, 10e-6   ## RC filter on duty_leg1 (tau = 10 ms).


def build(so_path: str):
    """Builds the stand-in circuit with the firmware block."""
    import pulsim

    b = pulsim.CircuitBuilder()
    # Sources for every measurement (1 V on a current node = 1 A).
    b.add_voltage_source("Vpv", "pv", "gnd", V_PV)
    b.add_voltage_source("Vbus", "bus", "gnd", V_BUS)
    b.add_sine_voltage_source("Vgrid", "vac", "gnd", 0.0, GRID_RMS * np.sqrt(2.0), GRID_HZ, 0.0)
    for node in ("il1", "il2", "iac", "relay"):
        b.add_voltage_source(f"V{node}", node, "gnd", 0.0)
    b.add_voltage_source("Vvalid", "vac_valid", "gnd", 1.0)
    for node in ("pv", "bus", "vac", "il1", "il2", "iac", "relay", "vac_valid"):
        b.add_resistor(f"R{node}", node, "gnd", 1e3)

    # Operator commands (stand in for the serial menu): IDLE, BUSREG from 0.2 s,
    # sync requested ('c'), 400 V bus reference.
    contract.add_command_timeline(b, "cmd_mode", 0.0, [(MODE_STEP_S, 2.0)])
    contract.add_command_timeline(b, "cmd_sync", 1.0)
    contract.add_command_timeline(b, "cmd_vref", 400.0)

    # An output driving a circuit element: RC low-pass on duty_leg1.
    b.add_resistor("Rf", contract.output_node("duty_leg1"), "duty_f", RC_R)
    b.add_capacitor("Cf", "duty_f", "gnd", RC_C)
    # C-blocks last: Pulsim resolves wires when a block is added, so every
    # component (including the firmware block's output loads) must exist before.
    handle = contract.add_firmware_block(b, so_path, {
        "VLow": ("v", "pv"), "ILow1": ("v", "il1"), "ILow2": ("v", "il2"),
        "VDCBus": ("v", "bus"), "VAC": ("v", "vac"), "IAC": ("v", "iac"),
        "relay_closed": ("v", "relay"), "vac_valid": ("v", "vac_valid"),
        "mode": ("v", "cmd_mode"), "sync_request": ("v", "cmd_sync"), "v_bus_ref": ("v", "cmd_vref"),
    }, sim_dt=SIM_DT)

    return b, handle


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    so_path = sys.argv[1]
    csv_out = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    import pulsim

    b, handle = build(so_path)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        t0 = time.perf_counter()
        res = pulsim.simulate(b, t_end=T_END, dt=SIM_DT)
        wall = time.perf_counter() - t0
    diverged = [str(w.message).split(" — ")[0] for w in caught if "reached" in str(w.message)]
    t = np.asarray(res.times)
    sig = {o: np.asarray(res.v(contract.output_node(o))) for o in contract.OUTPUTS}
    duty_f = np.asarray(res.v("duty_f"))

    if csv_out:
        cols = ["t"] + list(contract.OUTPUTS) + ["duty_f"]
        data = np.column_stack([t] + [sig[o] for o in contract.OUTPUTS] + [duty_f])
        np.savetxt(csv_out, data, delimiter=",", header=",".join(cols), comments="")

    fired = t >= contract.CONTROL_PERIOD_S  # first control step at 100 µs (t = 0 is skipped)
    expected_fires = int(round(T_END / contract.CONTROL_PERIOD_S)) + 1
    k = np.floor(t / contract.CONTROL_PERIOD_S + 1e-6).astype(int)
    before = t < MODE_STEP_S - 1e-9
    after = t >= MODE_STEP_S + contract.CONTROL_PERIOD_S

    with Criterion("L1-FWB-01.a", "Firmware block loads with the matching contract and controlInit succeeds") as c:
        c.check("contract version", contract.library_contract_version(so_path) == contract.CONTRACT_VERSION)
        c.check("status 0 at every sample", bool(np.all(sig["status"][fired] == 0.0)))

    with Criterion("L1-FWB-01.b", "Block runs every 100 µs and holds its outputs between runs") as c:
        c.near("block calls over 1.5 s (including the skipped t = 0 call)", handle.n_fires, expected_fires, 0)
        held = True
        for o in ("duty_leg1", "theta", "obs_rms"):
            v = sig[o]
            starts = np.r_[0, np.flatnonzero(np.diff(k)) + 1]
            for s, e in zip(starts, np.r_[starts[1:], len(v)]):
                if np.any(v[s:e] != v[s]):
                    held = False
                    break
        c.check("outputs constant inside each 100 µs period", held)

    with Criterion("L1-FWB-01.c", "Grid voltage arrives in volts: observer reads 230 V RMS and 50 Hz") as c:
        end = t > T_END - 0.1
        c.near("observer RMS (V)", float(np.mean(sig["obs_rms"][end])), GRID_RMS, 0.01 * GRID_RMS)
        c.near("observer frequency (Hz)", float(np.mean(sig["obs_freq"][end])), GRID_HZ, 0.05)

    with Criterion("L1-FWB-01.d", "Operator commands arrive within one control period: IDLE until 0.2 s, then BUSREG drives leg 1") as c:
        changes = t[(sig["mode_changed"] > 0.5)]
        c.check("exactly one mode change", len(np.unique(np.floor(changes / contract.CONTROL_PERIOD_S))) == 1)
        # Inputs are sampled at the block's next run: up to one control period of latency.
        c.range("mode change time (s)", float(changes[0]) if len(changes) else -1.0, MODE_STEP_S,
                MODE_STEP_S + 1.5 * contract.CONTROL_PERIOD_S)
        c.check("IDLE: legs off", bool(np.all(sig["en_leg1"][before] == 0) and np.all(sig["en_leg2"][before] == 0)))
        c.check("BUSREG below reference: leg 1 on", bool(np.all(sig["en_leg1"][after] == 1)))
        c.range("BUSREG leg 1 duty at end", float(sig["duty_leg1"][-1]), 0.05, 0.95)

    with Criterion("L1-FWB-01.e", "An output drives a circuit element: RC-filtered duty follows duty_leg1") as c:
        c.near("filtered duty at end", float(duty_f[-1]), float(sig["duty_leg1"][-1]), 0.01)

    with Criterion("L1-FWB-01.j", "No circuit node diverges or turns NaN") as c:
        c.check("Pulsim divergence warnings: " + ("; ".join(diverged) or "none"), not diverged)
        c.check("all outputs finite", all(bool(np.all(np.isfinite(sig[o]))) for o in contract.OUTPUTS))

    with Criterion("L1-FWB-01.f", "Bridge PWM and relay never requested") as c:
        c.check("en_bridge always 0", bool(np.all(sig["en_bridge"] == 0)))
        c.check("relay_request always 0", bool(np.all(sig["relay_request"] == 0)))

    with Criterion("L1-FWB-01.g", "Inverter synchronizes inside the simulation (1.5 s horizon)") as c:
        c.check("sync at end", bool(sig["sync"][-1] > 0.5))
    sync_t = t[np.argmax(sig["sync"] > 0.5)] if np.any(sig["sync"] > 0.5) else float("nan")
    info("L1-FWB-01.h", "Time to inverter synchronization in Pulsim (L0-APP-05.c on the host)", float(sync_t), "s")
    info("L1-FWB-01.i", "Wall-clock time per simulated second", wall / T_END, "s")
    return finish()


if __name__ == "__main__":
    sys.exit(main())
