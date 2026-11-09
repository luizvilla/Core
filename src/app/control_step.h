/*
 *
 * Copyright (c) 2021-present LAAS-CNRS
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
 * @file control_step.h
 * @brief Hardware-free control logic of the microinverter critical task.
 *
 * The critical task in main.cpp reads the sensors, calls controlStep() once
 * per control period, and applies the returned PWM commands. Everything
 * between those two hardware accesses lives here, so the same code runs on
 * the Spin board, in the L0 host tests and inside the L1 simulator
 * (docs/adr/0001-firmware-in-the-loop-via-pulsim.md).
 *
 * This file holds no hardware call. Keep it that way: logic added to the
 * critical task outside controlStep() is not covered by L0-APP or L1 tests.
 */
#ifndef CONTROL_STEP_H
#define CONTROL_STEP_H

#include <stdint.h>

#include "boost_converter/boost.h"
#include "grid_observer/gridObserver.h"
#include "singlePhaseInverterFollowing.h"

/** @brief Application timing and controller tuning. */
namespace app {

/** @brief Critical task period, in µs. */
inline constexpr uint32_t control_task_period = 100;
/** @brief Run the boost voltage/startup or MPPT cold loop every ten critical ticks. */
inline constexpr uint8_t cold_loop_divider = 10U;

/** @brief Minimum PWM duty cycle applied to every leg. */
inline constexpr float32_t duty_cycle_min = 0.005F;

/* Controller timing and initial tuning; validate gains/current limits on hardware. */
/** @brief Hot (current) loop period, in s. */
inline constexpr float32_t hot_loop_period = control_task_period * 1e-6F;
/** @brief Cold (voltage/MPPT) loop period, in s. */
inline constexpr float32_t cold_loop_period = hot_loop_period * cold_loop_divider;
/** @brief Initial VDCBus reference, in V (shared boost output capacitor feeding the DC/AC stage). */
inline constexpr float32_t initial_voltage_reference = 400.0F;
/*
 * Boost control from Andres Vazquez Sieber's bench-tested controller
 * (core-micro-verter-boost 43e249e), adopted by ADR 0003; values confirmed with
 * Andres on 2026-11-06 (worklog).
 */
/** @brief Maximum PWM duty cycle of the boost legs (Andres's bench value). */
inline constexpr float32_t duty_cycle_max = 0.60F;
/** @brief STARTUP total current ceiling, in A, converter 1 only (gentle bus charge). */
inline constexpr float32_t startup_current_max = 0.25F;
/** @brief Rated total input current, in A (400 W at about 36 V). */
inline constexpr float32_t rated_input_current = 11.0F;
/** @brief Leg 1 current limit, in A; leg 2 takes the excess. */
inline constexpr float32_t leg1_current_limit = 5.5F;
/** @brief Burst period on leg 1, in PWM periods (200 = 1 ms at 200 kHz). */
inline constexpr uint16_t boost_burst_period = 200U;
/** @brief Bus voltage above which leg 1 skips every period, in V. */
inline constexpr float32_t boost_full_burst_voltage = 440.0F;
/** @brief Bus voltage above which the boost trips (latched until IDLE), in V. */
inline constexpr float32_t boost_trip_voltage = 460.0F;
/** @brief Leg current above which the boost trips (latched until IDLE), in A. */
inline constexpr float32_t boost_trip_current = 8.0F;
/** @brief Voltage PID gain schedule: Kp = gain · VBusRef / VLow, in A/V. */
inline constexpr float32_t boost_voltage_gain = 2.6F / 6.0F;
/** @brief Input voltage range of the gain schedule, in V. */
inline constexpr float32_t boost_schedule_vin_min = 24.0F;
inline constexpr float32_t boost_schedule_vin_max = 48.0F;
/** @brief Bus voltage range of the gain schedule, in V. */
inline constexpr float32_t boost_schedule_vbus_min = 390.0F;
inline constexpr float32_t boost_schedule_vbus_max = 410.0F;
/** @brief Boost voltage PID: Ts, Kp (scheduled at run time; nominal 400 V / 36 V), Ti, Td, N, current bounds (A). */
inline constexpr PidParams boost_voltage_params = {
    cold_loop_period, boost_voltage_gain * 400.0F / 36.0F, 4.57e-3F, 0.0F, 0.0F, 0.0F, rated_input_current
};
/** @brief Boost current PID: Ts, Kp, Ti, Td, N, lower and upper duty bound. */
inline constexpr PidParams boost_current_params = {
    hot_loop_period, 0.002712F, 3.413e-4F, 0.0F, 0.0F, duty_cycle_min, duty_cycle_max
};

/** @brief Nominal grid frequency, in Hz. */
inline constexpr float32_t grid_frequency_hz = 50.0F;
/** @brief Nominal grid angular frequency, in rad/s. */
inline constexpr float32_t grid_angular_frequency = 2.0F * PI * grid_frequency_hz;
/** @brief Nominal grid peak voltage, in V. */
inline constexpr float32_t grid_voltage_peak = 230.0F * 1.41421356F;
/**
 * @brief Grid observer settings.
 *
 * Initial engineering settings, not a grid-code protection profile. Voltage and
 * frequency windows match the VSI's existing synchronization tolerances.
 * Validate thresholds, delays and SOGI/PLL response before grid operation.
 */
inline constexpr GridObserverConfig grid_observer_config = {
    hot_loop_period,
    230.0F * 0.9F, 230.0F * 1.1F,
    grid_frequency_hz * 0.95F, grid_frequency_hz * 1.05F,
    0.1F, 0.1F, 1.0F
};
/** @brief Invalidate the observer if no new VAC sample arrives for 20 ms. */
inline constexpr uint32_t grid_voltage_stale_ticks = 200U;

/*
 * Inverter bench session (docs/testing/bench/inverter-session.md), 2026-11-09.
 * Bench: DC source on the bus, grid through a series light bulb, relay on GPIO 22.
 */
/** @brief Grid periods of synchronized PWM before relay closure in the CONNECT stage. */
inline constexpr uint16_t inverter_settle_cycles = 10U;
/** @brief Open-loop stage: target output peak voltage (230 V RMS), in V. */
inline constexpr float32_t open_loop_voltage_peak = 230.0F * 1.41421356F;
/** @brief Open-loop stage: largest modulation index (keeps the duty inside 0.025-0.975). */
inline constexpr float32_t open_loop_m_max = 0.95F;
/** @brief Open-loop stage: modulation index ramp, per second (0 to full in about 1 s). */
inline constexpr float32_t open_loop_m_rate = 1.0F;
/** @brief INJECT stage: largest |d-axis current| reference, in A peak (series bulb: keep small). */
inline constexpr float32_t inject_current_max = 0.1F;
/** @brief Bench protection: |IAC| above this trips the inverter (latched until OFF), in A. */
inline constexpr float32_t bench_inverter_current_trip = 1.5F;
/** @brief Relay contact time: command held this long before the feedback changes, in control ticks (20 ms). */
inline constexpr uint16_t relay_contact_ticks = 200U;

}  // namespace app

/** @brief Latest sensor values, in physical units, held from the last valid sample. */
struct Measurements {
    float32_t i_low_1 = 0.0F;   ///< Boost leg 1 current (A).
    float32_t i_low_2 = 0.0F;   ///< Boost leg 2 current (A).
    float32_t v_low = 0.0F;     ///< PV-side voltage (V).
    float32_t v_dc_bus = 0.0F;  ///< DC bus voltage (V).
    float32_t v_ac = 0.0F;      ///< Instantaneous grid voltage (V).
    float32_t i_ac = 0.0F;      ///< Instantaneous bridge current (A).
    bool v_ac_updated = false;  ///< A new VAC sample arrived during this period.
    bool relay_closed = false;  ///< Relay feedback from the hardware loop (command held for the contact time).
};

/**
 * @brief Inverter bench stages (docs/testing/bench/inverter-session.md).
 * @details Values are the serial/scenario encoding: 1 keeps the earlier
 * "sync request" meaning (PWM_SETTLE hold, menu 'c').
 */
enum InverterStage : uint8_t {
    INV_OFF = 0,        ///< Inverter off: disconnect sequence, relay open, faults cleared once safe.
    INV_SETTLE = 1,     ///< Step 1: synchronized PWM held in PWM_SETTLE, relay open (menu 'c').
    INV_OPEN_LOOP = 2,  ///< Step 0: fixed 50 Hz modulation to 230 V RMS, no grid controller, relay open (menu 'o').
    INV_CONNECT = 3,    ///< Step 2: settle, close the relay, regulate zero current (GRID_CONNECTED) (menu 'g').
    INV_INJECT = 4      ///< Step 3: as CONNECT, then inject a fixed d-axis current (RUN) (menu 'n').
};

/** @brief Operator requests from the serial menu. */
struct OperatorCommands {
    boost_mode mode = IDLE;          ///< Requested boost mode.
    InverterStage inverter_stage = INV_OFF;  ///< Requested inverter bench stage.
    float32_t inject_current = 0.0F; ///< INJECT d-axis current reference, A peak (clamped to app::inject_current_max).
    float32_t v_bus_ref = app::initial_voltage_reference;  ///< VDCBus reference (V).
};

/** @brief PWM commands for one control period. */
struct ControlOutputs {
    bool boost_mode_changed = false;  ///< Mode changed this period: stop both boost legs before applying the rest.
    bool boost_pwm_update = false;    ///< Requested mode is not IDLE: apply boost duties and enables.
    BoostDutyCycles boost_duty;       ///< Boost leg duties.
    bool leg1_enable = false;         ///< Boost leg 1 PWM enable (when boost_pwm_update).
    bool leg2_enable = false;         ///< Boost leg 2 PWM enable (when boost_pwm_update).
    uint16_t burst_skip = 0U;         ///< Leg 1 PWM periods off per burst period; 0: burst generator stopped.
    uint16_t burst_period = 0U;       ///< Burst period, in PWM periods.
    bool boost_trip = false;          ///< Boost protection tripped (latched until IDLE).
    float32_t bridge_duty = 0.5F;     ///< Paired inverter duty to apply before enabling PWM.
    bool bridge_enable = false;       ///< Inverter bridge PWM request; hardware loop acknowledges application.
    bool relay_close = false;         ///< Grid relay command (GPIO 22); feedback comes back as Measurements::relay_closed.
    bool bench_trip = false;          ///< Bench current protection tripped (latched until the OFF stage).
    float32_t grid_angle = 0.0F;      ///< PLL grid angle (rad).
};

/** @brief Controllers and the state the control step keeps between periods. */
struct ControlState {
    BoostConverter boost;                       ///< Dual-leg boost controller.
    singlePhaseInverterFollowing inverter;      ///< Grid-following inverter controller.
    GridObserver grid_observer;                 ///< Grid voltage and frequency supervision.
    uint8_t cold_loop_tick = 0U;                ///< Periods since the last cold loop.
    boost_mode previous_mode = IDLE;            ///< Mode applied in the previous period.
    uint32_t grid_voltage_age_ticks = app::grid_voltage_stale_ticks;  ///< Periods since the last VAC sample.
    bool bridge_pwm_applied = false;            ///< Bridge PWM enable applied last period, acknowledged to the inverter.
    float32_t open_loop_m = 0.0F;               ///< Open-loop stage: current modulation index (ramped).
    float32_t open_loop_theta = 0.0F;           ///< Open-loop stage: oscillator angle (rad).
    bool bench_tripped = false;                 ///< Bench current protection latched (cleared in the OFF stage).
    bool open_loop_active = false;              ///< The open-loop stage drove the bridge last period.
};

/**
 * @brief Initializes and configures every controller with the application's tuning.
 * @param state Control state to initialize.
 * @return 0 on success, -1 if a controller rejected its configuration,
 * -2 if the boost burst, protection or gain schedule was rejected.
 */
int8_t controlInit(ControlState &state);

/**
 * @brief Runs the control logic for one control period.
 *
 * Applies mode requests, runs the boost cold loop every app::cold_loop_divider
 * periods and its hot loop every period, keeps the PLL running, updates the
 * grid observer (invalidated when VAC is stale for app::grid_voltage_stale_ticks),
 * and drives the inverter bench stage (InverterStage): OFF, SETTLE (PWM_SETTLE
 * held, relay open), OPEN_LOOP (fixed modulation, relay open), CONNECT (relay
 * closed after settling, zero current) and INJECT (fixed d-axis current).
 * Grid quality gates the grid stages. The hardware loop updates
 * bridge_pwm_applied after applying the returned commands, and reports the
 * relay through Measurements::relay_closed.
 *
 * @param state Control state, updated in place.
 * @param meas Latest measurements.
 * @param cmd Operator commands.
 * @return PWM commands to apply, in the order documented in ControlOutputs.
 */
ControlOutputs controlStep(ControlState &state, const Measurements &meas,
                           const OperatorCommands &cmd);

#endif  // CONTROL_STEP_H
