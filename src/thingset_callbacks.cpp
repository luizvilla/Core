/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

#include <string.h>

static uint8_t previous_mode;

void conf_mode_cb(enum thingset_callback_reason reason)
{
    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous_mode = mode;
            break;

        case THINGSET_CALLBACK_POST_WRITE:
            if (mode >= NUM_OF_MODES) {
                mode = previous_mode;
            }
            break;

        default:
            break;
    }
}

typedef struct
{
    float duty_cycle;
    char tracking_name[TRACKING_NAME_SIZE];
} leg_shadow_t;

static leg_shadow_t previous_legs[POWER_LEG_COUNT];

static void conf_leg_cb(enum thingset_callback_reason reason, uint8_t leg_index)
{
    power_leg_t &leg = power_legs[leg_index];
    leg_shadow_t &previous = previous_legs[leg_index];

    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous.duty_cycle = leg.duty_cycle;
            memcpy(previous.tracking_name, leg.tracking_name, TRACKING_NAME_SIZE);
            break;

        case THINGSET_CALLBACK_POST_WRITE: {
            if (leg.duty_cycle < 0.0f || leg.duty_cycle > 1.0f) {
                leg.duty_cycle = previous.duty_cycle;
            }

            if (strncmp(leg.tracking_name, previous.tracking_name,
                        TRACKING_NAME_SIZE) == 0) {
                break;
            }

            bool found = false;
            for (uint8_t i = 0; i < TRACKING_VAR_COUNT; ++i) {
                if (strcmp(leg.tracking_name, tracking_variables[i].name) == 0) {
                    leg.tracking_var = tracking_variables[i].address;
                    found = true;
                    break;
                }
            }

            if (!found) {
                memcpy(leg.tracking_name, previous.tracking_name, TRACKING_NAME_SIZE);
            }
            break;
        }

        default:
            break;
    }
}

void conf_leg_cb_0(enum thingset_callback_reason reason)
{
    conf_leg_cb(reason, 0);
}

void conf_leg_cb_1(enum thingset_callback_reason reason)
{
    conf_leg_cb(reason, 1);
}
