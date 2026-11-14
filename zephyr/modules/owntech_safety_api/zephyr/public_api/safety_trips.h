/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_trips.h
 * @brief Hardware trips (ADR 0009): comparator path (#18). Armed by the HRTIM
 *        driver's safety gate before the first PWM output is enabled.
 */

#ifndef SAFETY_TRIPS_H_
#define SAFETY_TRIPS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Configures, self-tests and locks every comparator trip. @return 0, or -1 (PWM must stay refused). */
int safety_trips_arm(void);
/** @brief @return true once armed. */
bool safety_trips_armed(void);
/** @brief @return HRTIM fault inputs armed (bit n = FLTn+1). */
uint32_t safety_trips_flt_mask(void);
/** @brief @return HRTIM fault inputs that tripped since the last clear (bit n = FLTn+1). */
uint32_t safety_trips_tripped(void);
/** @brief @return why arming failed, empty if it did not. */
const char *safety_trips_error(void);
/** @brief @return true while any armed comparator sees its limit exceeded. */
bool safety_trips_source_active(void);
/** @brief Clears the HRTIM fault flags and re-enables the fault interrupt. */
void safety_trips_clear(void);
/** @brief Read-back of every threshold; called once per control period. @return 0, or -1 (tripped). */
int safety_trips_check(void);
/** @brief @return number of comparator trips the build allocated. */
int safety_trips_count(void);
/**
 * @brief Tightens a comparator threshold, in the sensor's unit (tighten only).
 * @return 0; -1 loosening refused; -2 sensor without linear scaling; -3 out of the ADC range; -4 no comparator
 *         trip on that bound.
 */
int safety_trips_set_threshold(const char *sensor, bool high_bound, float value);
/** @brief @return the current DAC code of comparator trip index, -1 if none. */
int safety_trips_dac_code(int index);
/** @brief @return number of ADC-watchdog trips the build allocated. */
int safety_trips_watchdog_count(void);
/** @brief ADC (1 or 2) and channel watched by ADC-watchdog trip index. */
void safety_trips_watchdog_channel(int index, uint8_t *adc, uint8_t *channel);
/** @brief @return 1 if watchdog trip index watches a current sensor. */
int safety_trips_watchdog_is_current(int index);
/** @brief @return 1 self-test passed, 2 failed, 0 not configured yet, for watchdog trip index. */
int safety_trips_watchdog_state(int index);
/** @brief Waits for fresh conversions and @return true while any armed trip sees its limit exceeded.
 *         Clears the sticky ADC-watchdog flags first (used by clearFault). */
bool safety_trips_settle_and_check(void);
/** @brief Raises a real hardware fault: through an armed watchdog's external event (EEV1-3 switched to a forced
 *         TIMx TRGO), else an armed comparator's threshold to the extreme. @return 0 if the HRTIM fault flag rose. */
int safety_trips_force_fault(void);
/** @brief @return how the last forced fault was raised: 1 external event, 2 comparator, 0 not raised. */
int safety_trips_last_force_path(void);
/** @brief Writes every armed comparator threshold back to its current value (after a forced fault). */
void safety_trips_restore_thresholds(void);
/** @brief Text of a trip detail (fault inputs, or threshold read-back), e.g. "I1_LOW high limit (COMP1->FLT4)".
 *         @return length written. */
int safety_trips_describe(uint32_t detail, char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_TRIPS_H_ */
