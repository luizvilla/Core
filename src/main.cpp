/*
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
 * @brief  Minimal example exposing ThingSet over the Zephyr shell,
 *         reachable on a dedicated serial (USB-CDC) port.
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "pid.h"
#include "user_data_objects.h"

void setup_routine();
void loop_background_task();
void loop_critical_task();

static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
static constexpr float CONTROL_TASK_PERIOD_S = CONTROL_TASK_PERIOD_US * 1e-6f;
static const PidParams pid_params = {
    CONTROL_TASK_PERIOD_S,
    0.000215f,
    7.5175e-5f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
};

static Pid pid_leg1;
static Pid pid_leg2;

static void update_leg_duty(power_leg_t &leg, leg_t hardware_leg, Pid &pid)
{
    if (!leg.running) {
        return;
    }

    float duty = leg.duty_cycle;
    if (leg.buck || leg.boost) {
        duty = pid.calculateWithReturn(leg.reference_value, *leg.tracking_var);
    }
    if (leg.boost) {
        duty = 1.0f - duty;
    }

    shield.power.setDutyCycle(hardware_leg, duty);
    leg.duty_readback = duty;
}

void setup_routine()
{
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);
    shield.sensors.enableDefaultTwistSensors();

    uint32_t background_task_number = task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);

    pid_leg1.init(pid_params);
    pid_leg2.init(pid_params);

    task.startBackground(background_task_number);
    task.startCritical();
}

void loop_background_task()
{
    spin.led.toggle();

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
    if (meas_data != NO_VALUE) temp_1_value = meas_data;

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);
    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
    if (meas_data != NO_VALUE) temp_2_value = meas_data;

    /* blink_period_s is writable over the ThingSet shell (Config/wBlinkPeriod_s) */
    task.suspendBackgroundMs((uint32_t)(blink_period_s * 1000.0f));
}

void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;
    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;
    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;
    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;
    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high_value = meas_data;
    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high_value = meas_data;

    if (mode == POWER_ON && power_legs[0].enable && !power_legs[0].running) {
        shield.power.start(LEG1);
        power_legs[0].running = true;
    }
    if ((mode != POWER_ON || !power_legs[0].enable) && power_legs[0].running) {
        shield.power.stop(LEG1);
        power_legs[0].running = false;
        power_legs[0].v_max = 0.0f;
    }

    if (mode == POWER_ON && power_legs[1].enable && !power_legs[1].running) {
        shield.power.start(LEG2);
        power_legs[1].running = true;
    }
    if ((mode != POWER_ON || !power_legs[1].enable) && power_legs[1].running) {
        shield.power.stop(LEG2);
        power_legs[1].running = false;
        power_legs[1].v_max = 0.0f;
    }

    update_leg_duty(power_legs[0], LEG1, pid_leg1);
    update_leg_duty(power_legs[1], LEG2, pid_leg2);

    if (power_legs[0].running && V1_low_value > power_legs[0].v_max) {
        power_legs[0].v_max = V1_low_value;
    }
    if (power_legs[1].running && V2_low_value > power_legs[1].v_max) {
        power_legs[1].v_max = V2_low_value;
    }
}

int main(void)
{
    setup_routine();
    return 0;
}
