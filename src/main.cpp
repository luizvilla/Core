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
#include "user_data_objects.h"

void setup_routine();
void loop_background_task();
void loop_critical_task();

void setup_routine()
{
    shield.power.initBuck(ALL);
    shield.sensors.enableDefaultTwistSensors();

    uint32_t background_task_number = task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, 100);

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
}

int main(void)
{
    setup_routine();
    return 0;
}
