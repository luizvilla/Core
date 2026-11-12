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
 * @file control_step.cpp
 * @brief Hardware-free control logic of the microinverter critical task.
 *
 * Moved unchanged from loop_critical_task() and setup_routine() in main.cpp;
 * see docs/testing/worklog.md.
 */
#include "app/control_step.h"
#include "timing/timing_probe.h"  /* L0-TIM probes: empty unless TIMING_HARNESS (ADR 0005) */

#include <math.h>
#include "fast_code.h"

int8_t controlInit(ControlState &state)
{
    if (state.boost.init(IDLE, app::initial_voltage_reference, app::boost_voltage_params,
                         app::boost_current_params, app::leg1_current_limit) != 0 ||
        state.inverter.init(app::initial_voltage_reference, app::grid_voltage_peak,
                            app::grid_angular_frequency, app::hot_loop_period) != 0 ||
        !state.inverter.setCurrentPiGains(app::inverter_current_kp, app::inverter_current_ti) ||
        !state.inverter.setCurrentPiLimit(app::inverter_current_limit) ||
        !state.inverter.setCurrentSogiGain(app::inverter_current_sogi_gain) ||
        !state.inverter.setOutputPhaseAdvance(app::inverter_output_phase_advance) ||
        !state.grid_observer.init(app::grid_observer_config))
    {
        return -1;
    }
    state.inverter.setGridQuality(state.grid_observer.canOperate());
    /* Zero dwell holds synchronized PWM indefinitely without requesting relay closure. */
    state.inverter.setPwmSettleCycles(0);
    state.boost.setStartupCurrentMax(app::startup_current_max);
    state.boost.setVBusRef(app::initial_voltage_reference);
    if (state.boost.configureRippleControl(app::boost_ripple_frequency, app::boost_notch_bandwidth,
                                          app::boost_resonant_gain, app::boost_resonant_phase) != 0 ||
        state.boost.setBurst(app::boost_burst_period) != 0 ||
        state.boost.setProtection(app::boost_full_burst_voltage, app::boost_trip_voltage,
                                  app::boost_trip_current) != 0 ||
        state.boost.setVoltageGainSchedule(app::boost_voltage_gain,
                                           app::boost_schedule_vin_min, app::boost_schedule_vin_max,
                                           app::boost_schedule_vbus_min, app::boost_schedule_vbus_max) != 0)
    {
        return -2;
    }
    return 0;
}

FAST_CODE ControlOutputs controlStep(ControlState &state, const Measurements &meas,
                           const OperatorCommands &cmd)
{
    TIMING_PROBE(0);
    ControlOutputs out;

    /* Bench step 1c: the inverter aims vac_offset above the measured grid voltage, never below it. */
    const float32_t vac_offset = fminf(fmaxf(cmd.vac_offset, 0.0F), app::inverter_vac_offset_max);
    if (vac_offset != state.applied_vac_offset)  /* only on change: the critical task has little margin */
    {
        state.inverter.setFeedforwardGain(1.0F + vac_offset);
        state.applied_vac_offset = vac_offset;
    }

    if (state.grid_voltage_age_ticks < app::grid_voltage_stale_ticks) ++state.grid_voltage_age_ticks;
    if (meas.v_ac_updated) state.grid_voltage_age_ticks = 0U;

    state.boost.enableVoltageNotch(cmd.voltage_notch_enabled);
    state.boost.enableCurrentResonant(cmd.current_resonant_enabled);
    state.boost.setILow1(meas.i_low_1);
    state.boost.setILow2(meas.i_low_2);
    state.boost.setVLow(meas.v_low);
    state.boost.setVBus(meas.v_dc_bus);
    state.boost.setVBusRef(cmd.v_bus_ref);

    const boost_mode requested_mode = cmd.mode;
    if (requested_mode != state.previous_mode)
    {
        out.boost_mode_changed = true;
        state.boost.setMode(requested_mode);
        // Run the new mode's cold loop on this tick, using the measurements above.
        state.cold_loop_tick = app::cold_loop_divider - 1U;
        state.previous_mode = requested_mode;
    }

    if (++state.cold_loop_tick >= app::cold_loop_divider)
    {
        state.cold_loop_tick = 0U;
        state.boost.coldLoop();
    }
    TIMING_PROBE(1);
    out.boost_duty = state.boost.hotLoop();
    TIMING_PROBE(2);

    /* ---- Inverter bench stage (docs/testing/bench/inverter-session.md) ---- */
    const InverterStage stage = cmd.inverter_stage;
    const bool bus_valid = isfinite(meas.v_dc_bus) && meas.v_dc_bus > 0.0F;
    const bool follow_connect = stage == INV_FOLLOW_CONNECT;
    const bool grid_stage = stage == INV_SETTLE || stage == INV_CONNECT || stage == INV_INJECT || follow_connect;
    /* Keep the PLL running in idle, using a positive bus value for duty scaling; a grid
     * stage passes any bus value so the inverter faults on an invalid one. The open-loop
     * stage does not use the grid-following inverter, so it never faults it. */
    if (grid_stage || bus_valid) state.inverter.setVBus(meas.v_dc_bus);
    state.inverter.setGridConnected(meas.relay_closed);
    const vsi_mode_t vsi = state.inverter.getMode();
    const bool on_grid = vsi == VSI_GRID_CONNECT || vsi == VSI_GRID_CONNECTED || vsi == VSI_RUN ||
                         vsi == VSI_STOPPING || vsi == VSI_DISCONNECT;
    /* Relay closure only in CONNECT and INJECT; SETTLE holds PWM_SETTLE (zero dwell). */
    state.inverter.setPwmSettleCycles((stage == INV_CONNECT || stage == INV_INJECT || follow_connect) ? app::inverter_settle_cycles :
                                      (cmd.settle_count_test ? app::settle_count_test_cycles : 0U));
    /* Back to SETTLE from the grid: disconnect first (relay opens), then synchronize again. */
    if (grid_stage && !(stage == INV_SETTLE && on_grid) && !state.bench_tripped)
        state.inverter.requestConnect();
    else
        state.inverter.requestDisconnect();
    if (stage == INV_INJECT && !state.bench_tripped)
    {
        float32_t i_d = isfinite(cmd.inject_current) ? cmd.inject_current : 0.0F;
        i_d = fminf(fmaxf(i_d, -app::inject_current_max), app::inject_current_max);
        state.inverter.setIdqRef({i_d, 0.0F, 0.0F});
        state.inverter.requestRun();
    }
    else
    {
        state.inverter.requestStop();
        state.inverter.setIdqRef({0.0F, 0.0F, 0.0F});
    }
    if (stage == INV_OFF)
    {
        /* OFF clears the bench latch, and an inverter fault once the relay is open. */
        state.bench_tripped = false;
        if (vsi == VSI_FAULT && !meas.relay_closed) state.inverter.clearFault();
    }
    /* PWM start-up acknowledgement (single_phase_inverter PWM_WAIT/PWM_SETTLE): report
     * whether the hardware loop actually applied the inverter's own bridge PWM last period. */
    state.inverter.setPwmEnabled(state.bridge_pwm_applied && !state.open_loop_active);
    /* VAC copy (steps 1d and 2b): DC removed (filter always running, two operations), extrapolated over the delay. */
    const float32_t vac = isfinite(meas.v_ac) ? meas.v_ac : state.vac_prev;
    state.vac_dc += (vac - state.vac_dc) * (app::hot_loop_period / app::vac_follow_dc_tau);
    const bool copy_stage = follow_connect || stage == INV_VAC_FOLLOW;
    const float32_t vac_copy = copy_stage ? vac + app::vac_follow_advance * (vac - state.vac_prev) - state.vac_dc : 0.0F;
    /* Step 2b settings, applied only when entering or leaving the stage (the critical task has little margin):
     * current PI from the relay request, D-GFL-04 advance, blanking and connect window. */
    if (follow_connect != state.follow_connect_applied)
    {
        state.inverter.setCurrentLoopFromConnect(follow_connect);
        state.inverter.setOutputPhaseAdvance(follow_connect ? app::follow_connect_advance : app::inverter_output_phase_advance);
        state.inverter.setConnectionBlanking(follow_connect ? app::follow_connect_blanking : 0.0F);
        state.inverter.setConnectWindow(follow_connect ? app::follow_connect_window : 0.0F);
        if (!follow_connect) state.inverter.setExternalFeedforward(0.0F, false);
        state.follow_connect_applied = follow_connect;
    }
    if (follow_connect)  /* the VAC copy as feedforward; the inverter stops using it at the predicted contact */
        state.inverter.setExternalFeedforward(vac_copy, !state.bench_tripped);
    TIMING_PROBE(3);
    out.bridge_duty = state.inverter.calculateDuty(meas.v_ac, meas.i_ac);
    TIMING_PROBE(4);
    const clarke_t grid_voltage_ab = state.inverter.getVab();
    /* The observer judges one-period averages once the inverter's window is full (as its
     * synchronization does, #7): a real grid's harmonics ripple the instantaneous SOGI
     * amplitude and PLL frequency across the 207-253 V / 47.5-52.5 Hz limits. */
    if (state.inverter.isSyncWindowFull())
        state.grid_observer.updateEstimates(state.inverter.getVdqAverage().d * 0.70710678F,
                                            state.inverter.getwAverage() * (0.5F / PI),
                                            state.grid_voltage_age_ticks < app::grid_voltage_stale_ticks);
    else
        state.grid_observer.update(grid_voltage_ab.alpha, grid_voltage_ab.beta, state.inverter.getw(),
                               state.grid_voltage_age_ticks < app::grid_voltage_stale_ticks);
    /* Apply this tick's quality verdict before accepting any PWM request. */
    state.inverter.setGridQuality(state.grid_observer.canOperate());
    out.grid_angle = state.inverter.getTheta();
    TIMING_PROBE(5);

    out.boost_pwm_update = requested_mode != IDLE;
    out.leg1_enable = state.boost.isLeg1Enabled();
    out.leg2_enable = state.boost.isLeg2Enabled();
    out.burst_skip = state.boost.getBurstSkip();
    out.burst_period = state.boost.getBurstPeriod();
    out.boost_trip = state.boost.isTripped();
    /* Bridge and relay: the inverter's requests, or the open-loop stage. */
    out.bridge_enable = state.inverter.isPwmRequested();
    /* Open loop only with the relay open and the inverter off the grid (OFF, or FAULT: no PWM, no relay). */
    const vsi_mode_t vsi_now = state.inverter.getMode();
    const bool open_loop = stage == INV_OPEN_LOOP && !state.bench_tripped && bus_valid &&
                           (vsi_now == VSI_OFF || vsi_now == VSI_FAULT) && !meas.relay_closed;
    if (open_loop)
    {
        /* Step 0: fixed 50 Hz modulation, ramped to the 230 V RMS index, no feedback. */
        const float32_t target = fminf(app::open_loop_voltage_peak / meas.v_dc_bus, app::open_loop_m_max);
        const float32_t step = app::open_loop_m_rate * app::hot_loop_period;
        state.open_loop_m += fminf(fmaxf(target - state.open_loop_m, -step), step);
        state.open_loop_theta += app::grid_angular_frequency * app::hot_loop_period;
        if (state.open_loop_theta >= 2.0F * PI) state.open_loop_theta -= 2.0F * PI;
        out.bridge_duty = 0.5F + 0.5F * state.open_loop_m * sinf(state.open_loop_theta);
        out.bridge_enable = true;
    }
    /* Step 1d: VAC follower. Once the grid is qualified, the bridge copies the measured VAC (DC removed,
     * extrapolated over the actuation delay, offset of step 1c applied), ramped in like step 0. The inverter
     * controller stays OFF, so the relay is never requested. */
    const bool vac_follow = stage == INV_VAC_FOLLOW && !state.bench_tripped && bus_valid && state.grid_observer.canOperate() &&
                            (vsi_now == VSI_OFF || vsi_now == VSI_FAULT) && !meas.relay_closed;
    if (vac_follow)
    {
        const float32_t step = app::open_loop_m_rate * app::hot_loop_period;
        state.open_loop_m += fminf(fmaxf(1.0F - state.open_loop_m, -step), step);
        const float32_t predicted = vac_copy;
        const float32_t gain = 1.0F + fminf(fmaxf(cmd.vac_offset, 0.0F), app::inverter_vac_offset_max);
        out.bridge_duty = 0.5F + state.open_loop_m * gain * predicted / (2.0F * meas.v_dc_bus);
        out.bridge_enable = true;
    }
    state.vac_prev = vac;
    if (!open_loop && !vac_follow)
    {
        state.open_loop_m = 0.0F;
        state.open_loop_theta = 0.0F;
    }
    /* Bench protection: an AC current above the trip level stops the bridge (latched until OFF). */
    if ((out.bridge_enable || state.bridge_pwm_applied) && isfinite(meas.i_ac) &&
        fabsf(meas.i_ac) > app::bench_inverter_current_trip)
    {
        state.bench_tripped = true;
        state.inverter.tripFault(VSI_FAULT_EXTERNAL);
    }
    if (state.bench_tripped)
    {
        out.bridge_enable = false;
        state.open_loop_m = 0.0F;
    }
    state.open_loop_active = (open_loop || vac_follow) && out.bridge_enable;
    /* Relay: once the inverter requests the connection, issue the command app::relay_close_delay before a
     * grid-voltage zero crossing, so the contact closes near zero voltage. Locked, v = Vd·cos(theta): zero
     * crossings at theta = pi/2 (mod pi). The averaged frequency projects the angle over the closing time. */
    const bool was_latched = state.relay_command_latched;
    if (!state.inverter.isGridConnectionRequested())
    {
        state.relay_command_latched = false;
    }
    else if (!state.relay_command_latched)
    {
        if (app::relay_close_delay <= 0.0F)
        {
            state.relay_command_latched = true;
        }
        else
        {
            const float32_t w = state.inverter.getwAverage();
            float32_t phi = fmodf(state.inverter.getTheta() + w * app::relay_close_delay - 0.5F * PI, PI);
            if (phi < 0.0F) phi += PI;
            /* Within 1.5 periods' advance past the target: the angle steps at most about 1.1·w·Ts per period. */
            if (phi < 1.5F * w * app::hot_loop_period) state.relay_command_latched = true;
        }
    }
    /* Tell the inverter the relay was just commanded: it anticipates the contact relay_close_delay later (step 2b:
     * PLL frozen from now, feedforward held from the predicted contact, VAC copy stopped there). */
    if (state.relay_command_latched && !was_latched) state.inverter.notifyRelayCommand(app::relay_close_delay);
    out.relay_close = state.relay_command_latched;
    out.bench_trip = state.bench_tripped;
    TIMING_PROBE(6);
    return out;
}
