/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

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
