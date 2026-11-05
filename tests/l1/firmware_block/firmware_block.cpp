/*
 *
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file firmware_block.cpp
 * @brief Firmware block: the application's controlStep() exposed through the Pulsim C-Block ABI.
 *
 * Built into a shared library by build.sh and loaded by Pulsim with
 * add_c_block(lib=...). Pulsim calls pulsim_cblock_step() every 100 µs
 * (app::control_task_period) and holds the outputs in between, as the PWM
 * hardware holds a duty until the next update.
 *
 * Signal order follows the model ↔ firmware block contract (procedure §3).
 * contract.py holds the same order for the Python side; change both together
 * and bump CONTRACT_VERSION.
 *
 * The block adds no control logic: it converts doubles to the firmware's
 * types, calls controlInit() once and controlStep() every period, and copies
 * the results out. The only adaptations are those the critical task does
 * around controlStep() in main.cpp:
 * - a boost leg runs only when its enable is set and the mode is not IDLE
 *   (main.cpp stops both legs on a mode change and updates them only
 *   outside IDLE);
 * - commands arrive as analog levels and are rounded to the menu values.
 *
 * Simulator adaptation: Pulsim makes its first call at t = 0 before the
 * circuit is solved, so every input reads 0 (VAC absent, bus at 0 V). The
 * block skips that call (outputs 0, controllers untouched) and runs its first
 * control step at t = 100 µs, on solved measurements. Boot behaviour with a
 * missing first sample is tested at L0 (L0-APP-03.c), where the first tick is
 * set explicitly.
 */

#include "app/control_step.h"

#include <math.h>
#include <new>

/** @brief Contract version implemented by this block (procedure §3). */
#define CONTRACT_VERSION 1

/** @brief Input wires, in contract order. */
enum Input : int {
    IN_V_LOW = 0,         ///< PV-side voltage (V).
    IN_I_LOW_1,           ///< Boost leg 1 current (A).
    IN_I_LOW_2,           ///< Boost leg 2 current (A).
    IN_V_DC_BUS,          ///< DC bus voltage (V).
    IN_V_AC,              ///< Instantaneous grid voltage (V).
    IN_I_AC,              ///< Instantaneous bridge current (A).
    IN_RELAY_CLOSED,      ///< Relay feedback (> 0.5 = closed); unused by this application.
    IN_VAC_VALID,         ///< New VAC sample this period (> 0.5 = yes).
    IN_MODE,              ///< Operator mode 0–3 (IDLE, STARTUP, BUSREG, MPPT).
    IN_SYNC_REQUEST,      ///< Operator sync request (> 0.5 = yes).
    IN_V_BUS_REF,         ///< Operator VDCBus reference (V).
    IN_COUNT
};

/** @brief Output wires, in contract order. */
enum Output : int {
    OUT_DUTY_LEG1 = 0,    ///< Boost leg 1 duty.
    OUT_DUTY_LEG2,        ///< Boost leg 2 duty.
    OUT_EN_LEG1,          ///< Boost leg 1 PWM running (0/1).
    OUT_EN_LEG2,          ///< Boost leg 2 PWM running (0/1).
    OUT_DUTY_BRIDGE,      ///< Inverter leg duty computed by the controller.
    OUT_EN_BRIDGE,        ///< Inverter bridge PWM running (0/1).
    OUT_RELAY_REQUEST,    ///< Relay close request (0/1).
    OUT_THETA,            ///< PLL grid angle (rad). Log only.
    OUT_W,                ///< PLL angular frequency (rad/s). Log only.
    OUT_OBS_RMS,          ///< Observer RMS voltage (V). Log only.
    OUT_OBS_FREQ,         ///< Observer frequency (Hz). Log only.
    OUT_TRIP_REASON,      ///< Observer trip reason (GridTripReason as a number). Log only.
    OUT_INV_MODE,         ///< Inverter mode (vsi_mode_t as a number). Log only.
    OUT_SYNC,             ///< Inverter synchronized (getSync(), 0/1). Log only.
    OUT_BURST_PAUSED,     ///< Boost burst pause active (0/1). Log only.
    OUT_MODE_CHANGED,     ///< Boost mode changed this period (0/1). Log only.
    OUT_STATUS,           ///< 0 = running; -1/-2 = controlInit failed; -3 = wrong wire count.
    OUT_COUNT
};

/** @brief Per-block state owned by Pulsim through the opaque pointer. */
struct Block {
    ControlState control;  ///< Controllers and per-tick state, as in main.cpp.
    int8_t status = 0;     ///< controlInit() result, or -3 on a wire-count mismatch.
};

extern "C" {

/**
 * @brief Allocates the block and runs controlInit() with the application's tuning.
 * @param n_in Number of input wires; must equal IN_COUNT.
 * @param n_out Number of output wires; must equal OUT_COUNT.
 * @return The block state, or nullptr if allocation failed.
 */
void *pulsim_cblock_init(int n_in, int n_out)
{
    Block *block = new (std::nothrow) Block;
    if (block == nullptr) return nullptr;
    if (n_in != IN_COUNT || n_out != OUT_COUNT) {
        block->status = -3;
        return block;
    }
    block->status = controlInit(block->control);
    return block;
}

/**
 * @brief Runs one control period.
 * @param in Input wires, contract order.
 * @param n_in Number of inputs.
 * @param out Output wires, contract order.
 * @param n_out Number of outputs.
 * @param t Simulation time (s); the call at t <= 0 is skipped.
 * @param dt Block sample time (s), unused: the controllers use app::hot_loop_period.
 * @param state Pointer to the Block returned by pulsim_cblock_init().
 */
void pulsim_cblock_step(const double *in, int n_in, double *out, int n_out,
                        double t, double dt, void **state)
{
    (void)dt;
    Block *block = static_cast<Block *>(*state);
    if (block == nullptr || block->status != 0 || n_in != IN_COUNT || n_out != OUT_COUNT) {
        for (int k = 0; k < n_out; ++k) out[k] = 0.0;
        if (n_out > OUT_STATUS) out[OUT_STATUS] = block == nullptr ? -4.0 : block->status;
        return;
    }

    if (t <= 0.0) {
        // Pre-solve call: inputs are not yet valid (see the file comment).
        for (int k = 0; k < n_out; ++k) out[k] = 0.0;
        return;
    }

    Measurements meas;
    meas.v_low = static_cast<float32_t>(in[IN_V_LOW]);
    meas.i_low_1 = static_cast<float32_t>(in[IN_I_LOW_1]);
    meas.i_low_2 = static_cast<float32_t>(in[IN_I_LOW_2]);
    meas.v_dc_bus = static_cast<float32_t>(in[IN_V_DC_BUS]);
    meas.v_ac = static_cast<float32_t>(in[IN_V_AC]);
    meas.i_ac = static_cast<float32_t>(in[IN_I_AC]);
    meas.v_ac_updated = in[IN_VAC_VALID] > 0.5;

    OperatorCommands cmd;
    const double mode = fmin(fmax(round(in[IN_MODE]), 0.0), 3.0);
    cmd.mode = static_cast<boost_mode>(static_cast<int>(mode));
    cmd.sync_requested = in[IN_SYNC_REQUEST] > 0.5;
    cmd.v_bus_ref = static_cast<float32_t>(in[IN_V_BUS_REF]);

    ControlState &c = block->control;
    const ControlOutputs o = controlStep(c, meas, cmd);

    out[OUT_DUTY_LEG1] = o.boost_duty.leg1;
    out[OUT_DUTY_LEG2] = o.boost_duty.leg2;
    out[OUT_EN_LEG1] = (o.boost_pwm_update && o.leg1_enable) ? 1.0 : 0.0;
    out[OUT_EN_LEG2] = (o.boost_pwm_update && o.leg2_enable) ? 1.0 : 0.0;
    out[OUT_DUTY_BRIDGE] = o.bridge_duty;
    out[OUT_EN_BRIDGE] = o.bridge_enable ? 1.0 : 0.0;
    out[OUT_RELAY_REQUEST] = c.inverter.isGridConnectionRequested() ? 1.0 : 0.0;
    out[OUT_THETA] = o.grid_angle;
    out[OUT_W] = c.inverter.getw();
    out[OUT_OBS_RMS] = c.grid_observer.getVoltageRms();
    out[OUT_OBS_FREQ] = c.grid_observer.getFrequencyHz();
    out[OUT_TRIP_REASON] = static_cast<double>(c.grid_observer.getTripReason());
    out[OUT_INV_MODE] = static_cast<double>(c.inverter.getMode());
    out[OUT_SYNC] = c.inverter.getSync() ? 1.0 : 0.0;
    out[OUT_BURST_PAUSED] = c.boost.isBurstPaused() ? 1.0 : 0.0;
    out[OUT_MODE_CHANGED] = o.boost_mode_changed ? 1.0 : 0.0;
    out[OUT_STATUS] = 0.0;
}

/**
 * @brief Frees the block.
 * @param state Block returned by pulsim_cblock_init().
 */
void pulsim_cblock_term(void *state)
{
    delete static_cast<Block *>(state);
}

/**
 * @brief Contract version compiled into this library, checked by contract.py.
 * @return CONTRACT_VERSION.
 */
int firmware_block_contract_version(void)
{
    return CONTRACT_VERSION;
}

}  // extern "C"
