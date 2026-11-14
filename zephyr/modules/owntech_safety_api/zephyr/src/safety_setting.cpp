/*
 * Copyright (c) 2024-present LAAS-CNRS
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

/*
 * @date   2024
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @brief  This file contains the essentials functions to set safety limits
 */

/* Header */
#include "safety_setting.h"
#include "safety_internal.h"

/* Includes */

/* Safety API (ADR 0009): the software check reacts like every other trip */
#include "safety_supervision.h"
#include "safety_states.h"
#include "safety_mode.h"
#include "safety_shield.h"

/* OWNTECH APIs */
#include "nvs_storage.h"
#include "SpinAPI.h"
#include "ShieldAPI.h"

/* Zephyr */
#include <string.h>
#include "zephyr/kernel.h"

/* Defines */

/**
 * Defines the sensor numbers.
 */
#define SENSOR_COUNTER(node_id) +1
#define DT_SENSORS_NUMBER DT_FOREACH_STATUS_OKAY(shield_sensors, SENSOR_COUNTER)

/* Global variables */

/* sensors that need to be watched (true) / ignored (false) */
static bool sensor_watch[DT_SENSORS_NUMBER + 1];

/* threshold max for each sensor */
static float32_t sensor_threshold_max[DT_SENSORS_NUMBER + 1];

/* threshold min for each sensor */
static float32_t sensor_threshold_min[DT_SENSORS_NUMBER + 1];

/* Reaction type by default in open circuit mode */
static safety_reaction_t sensor_reaction = Open_Circuit;

/* sensor that went over/below the threshold (true) */
static bool sensor_errors[DT_SENSORS_NUMBER + 1];

/**
 * The purpose of safety_alert_counter is to have a delay when we detect a problem.
 * For example here we wait that safety_alert_counter = 5 before triggering
 * a safety alert, if the control task = 100µs then we wait 0.5ms before
 * enabling short-circuit and open-circuit mode.
 * Thus we avoid stopping everything because of transient surge of current or
 * voltage which are not really a problem.
 */
static uint8_t safety_alert_counter = 0;

/* The first sensor found out of its limits, for the fault record */
static uint8_t trip_sensor;
static bool trip_high;
static float32_t trip_value;

/* enable the safety API watch and action task */
static bool safety_enable = true;

/**
 * Private Functions
 */

/**
 * Public Functions
 */

/**
 * @brief Sets the sensors that need to be monitored
 */
int8_t safety_set_sensor_watch(sensor_t * safety_sensors,
                               uint8_t sensors_number)
{
    if (sensors_number > DT_SENSORS_NUMBER)
    {
        printk("ERROR: number of sensors superior to number of sensors defined \
                in device tree");

        return -1;
    }

    for (uint8_t i = 0; i < sensors_number; i++)
    {
        if (safety_sensors[i] < 1 || safety_sensors[i] > DT_SENSORS_NUMBER) return -1;
    }
    for (uint8_t i = 0; i < sensors_number; i++)
    {
        sensor_watch[safety_sensors[i]] = true;
    }

    return 0;
}

/**
 * @brief Gets if chosen sensor is being monitored or not
 */
bool safety_get_sensor_watch(sensor_t  safety_sensors)
{
    return sensor_watch[safety_sensors];
}

/**
 * @brief Unsets from the watching list the sensors that do not need
 *        to be monitored
 */
int8_t safety_unset_sensor_watch(sensor_t * safety_sensors,
                                 uint8_t sensors_number)
{
    if (sensors_number > DT_SENSORS_NUMBER)
    {
        printk("ERROR: number of sensors superior to number of sensors defined \
                in device tree");
        return -1;
    }

    for (uint8_t i = 0; i < sensors_number; i++)
    {
        if (safety_sensors[i] < 1 || safety_sensors[i] > DT_SENSORS_NUMBER) return -1;
    }
    for (uint8_t i = 0; i < sensors_number; i++)
    {
        sensor_watch[safety_sensors[i]] = false;
    }

    return 0;
}

/**
 * @brief Sets reaction type : Short-Circuit or Open-Circuit
 */
void safety_set_sensor_reaction(safety_reaction_t reaction)
{
    /* Compatibility wrapper (ADR 0009): the reaction is the safe state of
     * every leg, taken by the HRTIM on any trip. The HRTIM fixes it once the
     * leg's timer runs, so call this before the PWM is initialised. */
    safety_safe_state_t state = (reaction == Short_Circuit) ? SAFETY_SAFE_SHORT_LOW : SAFETY_SAFE_OPEN;
    bool late = false;
    for (int leg = 0; leg < safety_leg_count(); leg++)
    {
        if (safety_set_leg_safe_state(leg, state) == -1) late = true;
    }
    if (late)
    {
        printk("safety: setChannelReaction() too late: a leg's PWM timer already runs, its safe state "
               "is kept. Call it before the PWM is initialised, or use safe-state in the devicetree.\n");
        return;
    }
    sensor_reaction = reaction;
}

/**
 * @brief Gets reaction type : Short-Circuit or Open-Circuit
 */
safety_reaction_t safety_get_sensor_reaction()
{

    return sensor_reaction;
}

/* Tighten-only rule (ADR 0009): a software threshold may be tighter than the
 * shield's devicetree limit, never looser, except in development mode. All or
 * nothing: a refused value leaves every threshold of the call unchanged. */
static bool looser_than_devicetree(sensor_t sensor, float32_t value, bool high)
{
    float32_t lo, hi;
    if (!safety_dt_limits(sensor, &lo, &hi)) return false;   /* no board limit for this sensor */
    return high ? (value > hi) : (value < lo);
}

static int8_t safety_set_thresholds(sensor_t *sensors, float32_t *values, uint8_t number, bool high)
{
    if (number > DT_SENSORS_NUMBER)
    {
        printk("ERROR: number of sensors superior to number of sensors defined in device tree\n");
        return -1;
    }
    for (uint8_t i = 0; i < number; i++)
    {
        if (sensors[i] < 1 || sensors[i] > DT_SENSORS_NUMBER) return -1;
        if (!SAFETY_DEVELOPMENT_MODE && looser_than_devicetree(sensors[i], values[i], high))
        {
            printk("safety: %s threshold of sensor %d refused: looser than the board limit in the devicetree. "
                   "Limits can only be tightened (ADR 0009).\n", high ? "max" : "min", (int)sensors[i]);
            return -2;
        }
    }
    for (uint8_t i = 0; i < number; i++)
    {
        if (high) sensor_threshold_max[sensors[i]] = values[i];
        else      sensor_threshold_min[sensors[i]] = values[i];
    }
    return 0;
}

/**
 * @brief Sets sensor threshold
 */
int8_t safety_set_sensor_threshold_max(sensor_t *safety_sensors,
                                       float32_t *threshold,
                                       uint8_t sensors_number)
{
    return safety_set_thresholds(safety_sensors, threshold, sensors_number, true);
}

/**
 * @brief Sets sensor threshold
 */
int8_t safety_set_sensor_threshold_min(sensor_t *safety_sensors,
                                       float32_t *threshold,
                                       uint8_t sensors_number)
{
    return safety_set_thresholds(safety_sensors, threshold, sensors_number, false);
}

/**
 * @brief Returns the minimum threshold
 */
float32_t safety_get_sensor_threshold_min(sensor_t safety_sensor)
{
    return sensor_threshold_min[safety_sensor];
}

/**
 * @brief Returns the maximum threshold
 */
float32_t safety_get_sensor_threshold_max(sensor_t safety_sensor)
{
    return sensor_threshold_max[safety_sensor];
}

/**
 * @brief Returns if an error has been detected
 */
bool safety_get_sensor_error(sensor_t safety_sensor)
{
    return sensor_errors[safety_sensor];
}

/**
 * @brief Monitors measures that needs to be watched for safety purpose
 */
int8_t safety_watch()
{
    int8_t status = 0;

    /* sensor_t starts at 1 (0 is UNDEFINED_SENSOR) */
    for (uint8_t i = 1; i <= DT_SENSORS_NUMBER; i++)
    {
        if (sensor_watch[i])
        {
            float32_t measure =
                    shield.sensors.peekLatestValue(static_cast<sensor_t>(i));

            if (measure != -10000){
                bool high = measure > sensor_threshold_max[i];
                sensor_errors[i] = high || measure < sensor_threshold_min[i];
                if (sensor_errors[i] && status == 0)
                {
                    trip_sensor = i;   /* first sensor out of its limits, for the fault record */
                    trip_high = high;
                    trip_value = measure;
                }
            }
            if (sensor_errors[i])
                status = -1;
        }
    }

    return status;
}

/**
 * @brief Safety actions taken when we detect an error
 */
void safety_action()
{
    /* Same reaction as any other trip (ADR 0009): latch the fault, so the PWM
     * stays refused until clearFault(), and stop through the HRTIM fault
     * levels so every leg takes its safe state. Register-only: this runs in
     * the control-task interrupt (no GPIO API, no mutex). Once latched, the
     * outputs are already stopped. */
    if (safety_supervision_get_fault() != SAFETY_FAULT_NONE) return;
    uint32_t bits;
    memcpy(&bits, &trip_value, sizeof(bits));
    safety_supervision_latch_value(SAFETY_FAULT_TRIP,
                                   SAFETY_TRIP_SOFTWARE | (trip_high ? SAFETY_TRIP_SOFTWARE_HIGH : 0U)
                                   | (trip_sensor & 0x7FU),
                                   bits);
    safety_emergency_stop();
}

/**
 * @brief Enables the safety API fault detection task
 */
void safety_enable_task()
{
    safety_enable = true;
}

/**
 * @brief Disables the safety API fault detection task
 */
void safety_disable_task()
{
#if SAFETY_DEVELOPMENT_MODE
    printk("safety: DEVELOPMENT MODE: software check disabled; hardware trips and CPU supervision unchanged.\n");
    safety_enable = false;
#else
    printk("safety: disableSafetyApi() ignored: the software check can be disabled only in development mode "
           "(safety-mode = \"development\" in app.overlay, ADR 0009).\n");
#endif
}

/**
 * @brief Function that need to be put in the fast uninterruptible task.
 *        It monitors the measures from the ADC, and trigger safety warning.
 *        However, to avoid false triggering from transient phenomenon
 *        we wait for a delay with safety_alert_counter.
 */
int8_t safety_task()
{
    int8_t status = 0;

    if(safety_enable){
        status = safety_watch();

        if(status != 0)
        {
            safety_alert_counter++;
            if(safety_alert_counter > 4) safety_action();
            else status = 0;
        }
        else safety_alert_counter = 0;
    }

    return status;
}


/**
 * @brief Stores threshold value in the NVS
 */
int8_t safety_store_threshold_in_nvs(sensor_t sensor)
{

	uint8_t* buffer = (uint8_t*)k_malloc(1 + 23 + 4 + 4);

	uint8_t string_len = strlen((char*)(&buffer[1]));

	buffer[0]                                  = string_len;
    buffer[string_len + 1]                     = sensor;
	*((float32_t*)&buffer[string_len + 2])     = sensor_threshold_min[sensor];
	*((float32_t*)&buffer[string_len + 2 + 4]) = sensor_threshold_max[sensor];

	uint16_t sensor_ID = MEASURE_THRESHOLD | (sensor&0x0F);

	int ns = nvs_storage_store_data(sensor_ID,
                                    buffer,
                                    1 + string_len + 1 + 4 + 4);

	k_free(buffer);

	if (ns < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief Retrieves threshold value from the NVS
 */
int8_t  safety_retrieve_threshold_in_nvs(sensor_t sensor)
{
	/* Checks that parameters currently stored in NVS are
     * from the same version */
	uint16_t current_stored_version = nvs_storage_get_version_in_nvs();
	if (current_stored_version == 0)
	{
		return -1;
	}
	else if (current_stored_version != nvs_storage_get_current_version())
	{
		return -2;
	}

	uint16_t sensor_ID = MEASURE_THRESHOLD | (sensor&0x0F);

	int buffer_size = 1 + 23 + 4 + 4;
	uint8_t* buffer = (uint8_t*)k_malloc(buffer_size);

	int read_size = nvs_storage_retrieve_data(sensor_ID, buffer, buffer_size);

	int ret = 0;
	if (read_size > 0)
	{
		uint8_t string_len = buffer[0];

		/* Check that all required values match */
		if (sensor != buffer[string_len + 1])
		{
			ret = -3;
		}
		else
		{
            float32_t lo, hi;
            memcpy(&lo, &buffer[string_len + 2], sizeof(lo));
            memcpy(&hi, &buffer[string_len + 2 + 4], sizeof(hi));
            /* Tighten-only (ADR 0009): stored values looser than the
             * devicetree limit (an older firmware, another board) are ignored */
            if (!SAFETY_DEVELOPMENT_MODE &&
                (looser_than_devicetree(sensor, lo, false) || looser_than_devicetree(sensor, hi, true)))
            {
                ret = -5;
            }
            else
            {
                sensor_threshold_min[sensor] = lo;
                sensor_threshold_max[sensor] = hi;
            }
		}
	}
	else
	{
		ret = -4;
	}

	k_free(buffer);
	return ret;
}
