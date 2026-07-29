/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "scope_capture.h"

#include <math.h>
#include <stdint.h>

#include <ScopeMimicry.h>
#include <zephyr/sys/atomic.h>

#include "user_data_objects.h"

static_assert(SCOPE_SAMPLE_COUNT * SCOPE_CHANNEL_COUNT * sizeof(float) <=
              UINT16_MAX,
              "ScopeMimicry buffer size must fit its uint16_t size API");

bool scope_arm_request = false;
bool scope_trigger_request = false;
float scope_pretrigger_ratio = 0.2f;
uint16_t scope_decimation = 1;
uint8_t scope_state = SCOPE_STATE_IDLE;
uint16_t scope_sample_count = SCOPE_SAMPLE_COUNT;
uint16_t scope_channel_count = SCOPE_CHANNEL_COUNT;
uint32_t scope_sample_period_us = SCOPE_BASE_PERIOD_US;
float scope_capture_duration_ms =
    SCOPE_SAMPLE_COUNT * SCOPE_BASE_PERIOD_US / 1000.0f;
uint16_t scope_final_index = 0;
uint8_t scope_last_error = SCOPE_ERROR_NONE;

static ScopeMimicry scope(SCOPE_SAMPLE_COUNT, SCOPE_CHANNEL_COUNT);
static atomic_t arm_pending;
static atomic_t trigger_pending;
static bool trigger_level;
static uint16_t active_decimation = 1;
static uint16_t decimation_counter;

static float previous_pretrigger_ratio;
static uint16_t previous_decimation;

static bool scope_trigger(void)
{
    return trigger_level;
}

static bool scope_configuration_is_locked(void)
{
    return scope_state == SCOPE_STATE_ARMED ||
           scope_state == SCOPE_STATE_TRIGGERED ||
           scope_state == SCOPE_STATE_STREAMING;
}

void scope_capture_init(void)
{
    scope.connectChannel(V1_low_value, "V1Low_V");
    scope.connectChannel(V2_low_value, "V2Low_V");
    scope.connectChannel(V_high_value, "VHigh_V");
    scope.connectChannel(I1_low_value, "I1Low_A");
    scope.connectChannel(I2_low_value, "I2Low_A");
    scope.connectChannel(I_high_value, "IHigh_A");
    scope.connectChannel(power_legs[0].duty_readback, "Duty1");
    scope.connectChannel(power_legs[1].duty_readback, "Duty2");
    scope.set_trigger(scope_trigger);
    scope.set_delay(scope_pretrigger_ratio);
}

void scope_capture_process(void)
{
    if (atomic_cas(&arm_pending, 1, 0)) {
        active_decimation = scope_decimation;
        scope_sample_period_us =
            SCOPE_BASE_PERIOD_US * static_cast<uint32_t>(active_decimation);
        scope_capture_duration_ms =
            SCOPE_SAMPLE_COUNT * scope_sample_period_us / 1000.0f;
        decimation_counter = 0;
        trigger_level = false;
        atomic_clear(&trigger_pending);
        scope.set_delay(scope_pretrigger_ratio);
        scope.start();
        scope_final_index = 0;
        scope_last_error = SCOPE_ERROR_NONE;
        scope_state = SCOPE_STATE_ARMED;
    }

    if (scope_state != SCOPE_STATE_ARMED &&
        scope_state != SCOPE_STATE_TRIGGERED) {
        return;
    }

    if (scope_state == SCOPE_STATE_ARMED &&
        atomic_cas(&trigger_pending, 1, 0)) {
        trigger_level = true;
        scope_state = SCOPE_STATE_TRIGGERED;
    }

    ++decimation_counter;
    if (decimation_counter < active_decimation) {
        return;
    }
    decimation_counter = 0;

    const uint16_t acquisition_state = scope.acquire();
    if (trigger_level) {
        trigger_level = false;
    }

    if (acquisition_state == 2) {
        scope_final_index = scope.get_final_idx();
        scope_state = SCOPE_STATE_READY;
    }
}

void scope_config_cb(enum thingset_callback_reason reason)
{
    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous_pretrigger_ratio = scope_pretrigger_ratio;
            previous_decimation = scope_decimation;
            break;

        case THINGSET_CALLBACK_POST_WRITE: {
            const bool configuration_changed =
                scope_pretrigger_ratio != previous_pretrigger_ratio ||
                scope_decimation != previous_decimation;

            if (configuration_changed && scope_configuration_is_locked()) {
                scope_pretrigger_ratio = previous_pretrigger_ratio;
                scope_decimation = previous_decimation;
                scope_last_error = SCOPE_ERROR_INVALID_STATE;
            }
            else {
                if (!isfinite(scope_pretrigger_ratio) ||
                    scope_pretrigger_ratio < 0.0f ||
                    scope_pretrigger_ratio > 0.9f) {
                    scope_pretrigger_ratio = previous_pretrigger_ratio;
                    scope_last_error = SCOPE_ERROR_INTERNAL;
                }

                if (scope_decimation < SCOPE_DECIMATION_MIN ||
                    scope_decimation > SCOPE_DECIMATION_MAX) {
                    scope_decimation = previous_decimation;
                    scope_last_error = SCOPE_ERROR_INVALID_DECIMATION;
                }
            }

            if (scope_arm_request) {
                if (scope_state == SCOPE_STATE_STREAMING) {
                    scope_last_error = SCOPE_ERROR_INVALID_STATE;
                }
                else {
                    atomic_set(&arm_pending, 1);
                }
                scope_arm_request = false;
            }

            if (scope_trigger_request) {
                if (scope_state == SCOPE_STATE_ARMED) {
                    atomic_set(&trigger_pending, 1);
                }
                else {
                    scope_last_error = SCOPE_ERROR_INVALID_STATE;
                }
                scope_trigger_request = false;
            }
            break;
        }

        default:
            break;
    }
}
