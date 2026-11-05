## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file contract.py
## @brief Python side of the model ↔ firmware block contract: signal order and wiring helper.
## @see docs/testing/test-procedure.md §3, firmware_block.cpp, README.md in this folder
"""Model ↔ firmware block contract, Python side.

INPUTS and OUTPUTS list the C-Block wires in the order firmware_block.cpp
expects. They must change together with the enums in firmware_block.cpp and
with the table in the procedure (§3); CONTRACT_VERSION is checked against the
library at load time.

add_firmware_block() wires the block into a Pulsim CircuitBuilder:

    handle = add_firmware_block(builder, so_path, {
        "VLow": ("v", "pv"), "VDCBus": ("v", "bus"), ...   # every input
    })
    res = pulsim.simulate(builder, t_end, dt)
    duty = res.v(output_node("duty_leg1"))
"""

from __future__ import annotations

import ctypes
from pathlib import Path

## Contract version implemented here (procedure §3).
CONTRACT_VERSION = 1

## Block sample time: the critical task period (app::control_task_period).
CONTROL_PERIOD_S = 100e-6

## Input wires, in firmware_block.cpp order.
INPUTS = (
    "VLow", "ILow1", "ILow2", "VDCBus", "VAC", "IAC",
    "relay_closed", "vac_valid",
    "mode", "sync_request", "v_bus_ref",
)

## Output wires, in firmware_block.cpp order.
OUTPUTS = (
    "duty_leg1", "duty_leg2", "en_leg1", "en_leg2",
    "duty_bridge", "en_bridge", "relay_request",
    "theta", "w", "obs_rms", "obs_freq", "trip_reason",
    "inv_mode", "sync", "burst_paused", "mode_changed", "status",
)

## Load on each output node. Pulsim needs every controlled-source output to be
## loaded: an unloaded output node makes the circuit matrix singular and node
## voltages diverge without an error.
OUTPUT_LOAD_OHMS = 1e6

## Prefix of the output node names.
NODE_PREFIX = "fwb_"


def add_command_timeline(builder, node: str, initial: float, steps=(), name: str | None = None) -> None:
    """Drives a node with a piecewise-constant command (mode, sync request, bus reference).

    Built from ordinary sources, not a C-block: Pulsim 2.0.0 cannot combine
    two C-blocks in one circuit (their output vectors are summed with
    different lengths), so the firmware block must stay the only one.

    @param builder Pulsim CircuitBuilder.
    @param node Node to drive, read by the firmware block as a ("v", node) input.
    @param initial Value from t = 0.
    @param steps Sequence of (time_s, value) changes, in increasing time.
    @param name Prefix for the source names (default: the node name).
    """
    prefix = name or node
    builder.add_resistor(f"{prefix}_load", node, "gnd", OUTPUT_LOAD_OHMS)
    level, upper = initial, node
    for k, (t_step, value) in enumerate(steps):
        lower = f"{prefix}_s{k}"
        # One step source per change, stacked in series: v(node) = initial + sum of deltas.
        builder.add_pulse_voltage_source(f"{prefix}_step{k}", upper, lower, 0.0, value - level,
                                         float(t_step), 1e9)
        level, upper = value, lower
    builder.add_voltage_source(f"{prefix}_init", upper, "gnd", float(initial))


def output_node(name: str) -> str:
    """Circuit node carrying a block output (voltage to gnd = value)."""
    if name not in OUTPUTS:
        raise KeyError(f"unknown firmware block output {name!r}; see contract.OUTPUTS")
    return NODE_PREFIX + name


def library_contract_version(so_path: str | Path) -> int:
    """Contract version compiled into a firmware block library."""
    lib = ctypes.CDLL(str(so_path))
    fn = lib.firmware_block_contract_version
    fn.restype = ctypes.c_int
    return int(fn())


def add_firmware_block(builder, so_path: str | Path, inputs: dict, name: str = "FWB",
                       sim_dt: float | None = None):
    """Adds the firmware block to a Pulsim circuit.

    Call it after the plant is complete and before any other C-block: it adds
    a load resistor on every output, and Pulsim resolves the wires of a
    C-block when the block is added.

    @param builder Pulsim CircuitBuilder with the plant already built.
    @param so_path Shared library from build.sh.
    @param inputs Mapping of every name in INPUTS to a Pulsim input wire,
           ("v", node) or ("i", branch).
    @param name Block name.
    @param sim_dt Simulation step, passed to Pulsim's sample-time check.
    @return The Pulsim CBlockHandle.
    @exception ValueError on a missing or unknown input, or a contract mismatch.
    """
    import pulsim

    missing = [k for k in INPUTS if k not in inputs]
    unknown = [k for k in inputs if k not in INPUTS]
    if missing or unknown:
        raise ValueError(f"firmware block inputs: missing {missing}, unknown {unknown}")
    version = library_contract_version(so_path)
    if version != CONTRACT_VERSION:
        raise ValueError(f"contract mismatch: library v{version}, contract.py v{CONTRACT_VERSION}")
    for out in OUTPUTS:
        builder.add_resistor(f"{name}_load_{out}", output_node(out), "gnd", OUTPUT_LOAD_OHMS)
    return pulsim.add_c_block(
        builder,
        inputs=[inputs[k] for k in INPUTS],
        outputs=[("v", output_node(o), "gnd") for o in OUTPUTS],
        dt=CONTROL_PERIOD_S,
        lib=str(so_path),
        name=name,
        sim_dt=sim_dt,
    )
