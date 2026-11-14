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
 * @date 2024
 *
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 *
 * @brief Public class to use safety function
 *
 */

#ifndef SAFETYAPI_H_
#define SAFETYAPI_H_

#include "arm_math.h"
#include "ShieldAPI.h"
#include "../src/safety_enum.h"
#include "safety_supervision.h"
#include "safety_trips.h"
#include "safety_states.h"
#include "safety_mode.h"


class SafetyAPI{

    public:

    /**
     * @brief Initializes thresholds min/max with the default value
     *        from the device tree. 
     *        
     *        If values were stored and found in the flash
     *        they will be used instead. 
     *        
     *        All the sensors will be monitored
     *        to detect potential faults in voltage/current.
     */
    void initShield();

    /**
     * @brief Initializes thresholds min/max with the default value
     *        from the device tree. 
     * 
     *        If values were stored and found in the flash
     *        they will be used instead. 
     * 
     *        All the sensors will be monitored
     *        to detect potential faults in voltage/current.
     *
     * @param sensors_watch A list of the sensors to watch. The variables names 
     *                       can be:  
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param sensors_watch_number The number of sensors present
     *                             in the list sensors_watch.
     *
     */
    void initShield(sensor_t* sensors_watch, uint8_t sensors_watch_number);

    /**
     * @brief Enables the monitoring of the selected sensors for safety.
     *
     * @param sensors_watch A list of the sensors to watch. The variables names 
     *                       can be:  
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param sensors_watch_number The number of sensors present
     *                             in the list sensors_watch.
     *
     * @return `0` if successful, or `-1` if there was an error
     */
    int8_t setChannelWatch(sensor_t* sensors_watch,
                           uint8_t sensors_watch_number);

    /**
     * @brief Disables the monitoring of the selected sensors for safety.
     *
     * @param sensors_watch A list of the sensors to watch. The variables names 
     *                       can be:  
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param sensors_watch_number The number of sensors present
     *                             in the list sensors_watch.
     *
     * @return `0` if successful, or `-1` if there was an error
     */
    int8_t unsetChannelWatch(sensor_t* sensors_watch,
                             uint8_t sensors_watch_number);

    /**
     * @brief check if a sensor is being monitored or not.
     *
     * @param sensors_watch the sensor to check within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @return `true` if the sensor is being monitored, `false` if not
     */
    bool getChannelWatch(sensor_t  sensors_watch);

    /**
     * @brief Set the reaction to do if an error has been detected. 
     *
     * @param sensors_reaction the reaction to do: 
     * 
     * - `Open_Circuit`: both switches are opened and no power flows to the 
     *                   output 
     * 
     * - `Short_Circuit`: high-side switch is opened, and low-side is 
     *                    closed maintaining the output in short-circuit.
     *                    Useful in motor control.
     *
     * Compatibility wrapper (ADR 0009): sets the safe state of every leg,
     * which the HRTIM applies on any trip, hardware or software. Call it
     * before the PWM is initialised: once a leg's timer runs, its safe state
     * is fixed and the call is ignored (with a message). setSafeState() sets
     * one leg; the devicetree property safe-state is the usual way.
     */
    void setChannelReaction(safety_reaction_t sensors_reaction);

    /**
     * @brief returns the reaction to do when encountering an error.
     *
     * @return Open_Circuit or Short_Circuit
     */
    safety_reaction_t getChannelReaction();

    /**
     * @brief Set the maximum threshold for the sensors present
     *        in the list safety_sensors.
     *
     * @param sensors_threshold A list of the sensors to watch. The variables names 
     *                       can be:  
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param threshold_max A list of the maximum threshold to apply
     *                      to the sensors in safety_sensors.
     * 
     * @param sensors_threshold_number The number of sensors present
     *                                 in the list sensor_threshold
     *
     * Software check only (5 control periods). A threshold can be tighter
     * than the board limit in the devicetree, never looser, except in
     * development mode (ADR 0009). All or nothing: when one value is
     * refused, no threshold of the call changes.
     *
     * @return `0` if successful, `-1` for a bad sensor or count, `-2` when a
     *         value is looser than the board limit.
     */
    int8_t setChannelThresholdMax(sensor_t *sensors_threshold,
                                  float32_t *threshold_max,
                                  uint8_t sensors_threshold_number);

    /**
     * @brief Set the minimum threshold for the sensors present
     *        in the list safety_sensors.
     *
     * @param sensors_threshold A list of the sensors to watch. The variables names 
     *                       can be:  
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param threshold_min A list of the minimum threshold to apply
     *                      to the sensors in safety_sensors.
     * 
     * @param sensors_threshold_number The number of sensors present
     *                                 in the list sensor_threshold
     *
     * Software check only (5 control periods). A threshold can be tighter
     * than the board limit in the devicetree, never looser, except in
     * development mode (ADR 0009). All or nothing: when one value is
     * refused, no threshold of the call changes.
     *
     * @return `0` if successful, `-1` for a bad sensor or count, `-2` when a
     *         value is looser than the board limit.
     */
    int8_t setChannelThresholdMin(sensor_t *sensors_threshold,
                                  float32_t *threshold_min,
                                  uint8_t sensors_threshold_number);

    /**
     * @brief Get the maximum threshold of the selected sensor
     *
     * @param sensors_threshold the sensor to check within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     *
     * @return the maximum threshold
     */
    float32_t getChannelThresholdMax(sensor_t sensors_threshold);

    /**
     * @brief Get the minimum threshold of the selected sensor
     *
     * @param sensors_threshold the sensor to check within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @return The minimum threshold
     */
    float32_t getChannelThresholdMin(sensor_t sensors_threshold);

    /**
     * @brief Check if the sensor faced an error (went over/under threshold)
     *
     * @param sensors_error the sensor to check within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     *
     * @return True if the sensor faced an error, false if not
     */
    bool getChannelError(sensor_t sensors_error);


    /**
     * @brief Enables the software check (the default).
     */
    void enableSafetyApi();

    /**
     * @brief Disables the software check, in development mode only (ADR 0009).
     *        In any other build the call is ignored with a message. The
     *        hardware trips and CPU supervision are never affected.
     */
    void disableSafetyApi();
    /**
     * @brief Store the current minimum and maximum threshold
     *        in the flash (non volatile memory)
     *
     * @param sensor_threshold_store the sensor to store the threshold in the 
     *                               NVS within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     * 
     * @param sensor_threshold_store The sensor for which we store
     *                               the threshold in the NVS
     *
     * @return `0` if parameters were correctly stored, 
     *         `-1` if there was an error.
     *
     */
    int8_t storeThreshold(sensor_t sensor_threshold_store);
    /**
     * @brief Retrieves the current minimum and maximum threshold stored in 
     *        the flash (non volatile memory)
     *
     * @param sensor_threshold_retrieve the sensor to retrieve the threshold 
     *                           stored in the NVS within the possible names:
     * 
     * `V1_LOW`,`V2_LOW`, `V_HIGH`, `I1_LOW`,`I2_LOW`,`I_HIGH`, `TEMP_SENSOR`,
     * `EXTRA_MEAS`, `ANALOG_COMM` 
     *
     * @return `0` if parameters were correctly retrieved, negative value 
     *         if there was an error:
     * 
     * - `-1`: NVS is empty
     * 
     * - `-2`: NVS contains data, but their version doesn't
     *                                      match current version
     * 
     * - `-3`: NVS data is corrupted
     * 
     * - `-4`: NVS contains data, but not for the requested sensor
     *
     * - `-5`: the stored thresholds are looser than the board limit in the
     *         devicetree: ignored, the current thresholds are kept (ADR 0009)
     * 
     */
    int8_t retrieveThreshold(sensor_t sensor_threshold_retrieve);

    /**
     * @brief Returns the fault latched by CPU supervision (watchdog or fatal
     *        error, ADR 0009). While a fault is latched, PWM outputs cannot
     *        be enabled, even after a reset.
     *
     * @return SAFETY_FAULT_NONE, SAFETY_FAULT_FATAL_ERROR, SAFETY_FAULT_WATCHDOG,
     *         SAFETY_FAULT_TRIP (hardware trip or software check) or SAFETY_FAULT_SELFTEST.
     */
    safety_fault_t getFault();

    /**
     * @brief Clears the latched fault. Refused while any PWM output is enabled
     *        or a limit is still exceeded.
     *
     * @return 0 when cleared, -1 when refused, -2 when only a reboot clears it
     *         (a self-test failure, or a second fault soon after a clear).
     */
    int8_t clearFault();

    /**
     * @brief Tightens the hardware trip thresholds of a sensor, in its unit
     *        (ADR 0009). Only tightening is accepted: a lower high limit, a
     *        higher low limit than the current ones. The devicetree limit is
     *        the loosest a board ever gets.
     *
     * @param sensor  sensor (e.g. I1_LOW)
     * @param low     new low limit
     * @param high    new high limit
     * @return 0 when applied to every hardware bound of the sensor (a bound
     *         with no comparator is left as it is); -1 when a bound would be
     *         loosened (nothing changed on that bound); -3 when a limit is out
     *         of the sensor's measurement range.
     */
    int8_t setTripThreshold(sensor_t sensor, float32_t low, float32_t high);

    /**
     * @brief Sets a leg's safe state (ADR 0009): the state of its switches on
     *        any trip. Only before the leg is initialised (the HRTIM fixes it
     *        once its timer runs); the devicetree property safe-state is the
     *        usual way.
     * @param leg   LEG1, LEG2, ... (devicetree order)
     * @param state SAFETY_SAFE_OPEN, SAFETY_SAFE_SHORT_LOW or SAFETY_SAFE_SHORT_HIGH
     * @return 0; -1 too late; -2 bad leg or state.
     */
    int8_t setSafeState(int leg, safety_safe_state_t state);

    /**
     * @brief Writes the text of the latched fault into buf, e.g.
     *        "hardware trip: I1_LOW high limit (COMP1->FLT4); clear: PWM off, clearFault()"
     *        or "software trip: V_HIGH 52.3 above high limit; clear: ...".
     * @return the text length.
     */
    int getFaultDescription(char *buf, size_t size);

    /**
     * @brief @return true in a development build (safety-mode = "development"
     *        in app.overlay, ADR 0009).
     */
    bool isDevelopmentMode() { return SAFETY_DEVELOPMENT_MODE != 0; }

#if SAFETY_DEVELOPMENT_MODE
    /**
     * @brief Development mode only: the comparator and ADC-watchdog trips are
     *        not armed when the PWM starts. CPU supervision stays on. Call it
     *        before the first PWM output is enabled.
     * @return 0; -1 too late (armed trips stay locked until the next reset).
     */
    int8_t disarm() { return (int8_t)safety_disarm(); }
#else
    /* In a production build disarm() does not compile (ADR 0009). */
    template <typename T = void>
    int8_t disarm()
    {
        static_assert(sizeof(T) == 0,
                      "safety.disarm() exists only in development mode: add a safety-config node with "
                      "safety-mode = \"development\" to app.overlay (ADR 0009). Never on a deployed converter.");
        return -1;
    }
#endif


};

extern SafetyAPI safety;

#endif /* SAFETYAPI_H_ */
