/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#ifndef SCOPE_CAPTURE_H_
#define SCOPE_CAPTURE_H_

#include <stdbool.h>
#include <stdint.h>

#include <thingset.h>

static constexpr uint16_t SCOPE_SAMPLE_COUNT = 1024;
static constexpr uint16_t SCOPE_CHANNEL_COUNT = 8;
static constexpr uint32_t SCOPE_BASE_PERIOD_US = 100;
static constexpr uint16_t SCOPE_DECIMATION_MIN = 1;
static constexpr uint16_t SCOPE_DECIMATION_MAX = 100;

typedef enum : uint8_t
{
    SCOPE_STATE_IDLE = 0,
    SCOPE_STATE_ARMED,
    SCOPE_STATE_TRIGGERED,
    SCOPE_STATE_READY,
    SCOPE_STATE_STREAMING,
    SCOPE_STATE_ERROR,
} scope_state_t;

typedef enum : uint8_t
{
    SCOPE_ERROR_NONE = 0,
    SCOPE_ERROR_INVALID_STATE,
    SCOPE_ERROR_INVALID_DECIMATION,
    SCOPE_ERROR_TRANSFER,
    SCOPE_ERROR_INTERNAL,
} scope_error_t;

extern bool scope_arm_request;
extern bool scope_trigger_request;
extern float scope_pretrigger_ratio;
extern uint16_t scope_decimation;
extern uint8_t scope_state;
extern uint16_t scope_sample_count;
extern uint16_t scope_channel_count;
extern uint32_t scope_sample_period_us;
extern float scope_capture_duration_ms;
extern uint16_t scope_final_index;
extern uint8_t scope_last_error;

void scope_capture_init(void);
void scope_capture_process(void);
void scope_config_cb(enum thingset_callback_reason reason);

#endif /* SCOPE_CAPTURE_H_ */
