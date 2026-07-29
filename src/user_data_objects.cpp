/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

#include <thingset.h>
#include <thingset/sdk.h>

float V1_low_value;
float V2_low_value;
float V_high_value;
float I1_low_value;
float I2_low_value;
float I_high_value;
float temp_1_value;
float temp_2_value;
float meas_data;

float blink_period_s = 1.0f;

THINGSET_ADD_GROUP(ID_ROOT, ID_MEAS, "Measurements", THINGSET_NO_CALLBACK);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V1_LOW, "rV1Low_V", &V1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V2_LOW, "rV2Low_V", &V2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V_HIGH, "rVHigh_V", &V_high_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I1_LOW, "rI1Low_A", &I1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I2_LOW, "rI2Low_A", &I2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I_HIGH, "rIHigh_A", &I_high_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP1, "rTemp1_degC", &temp_1_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP2, "rTemp2_degC", &temp_2_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);

THINGSET_ADD_GROUP(ID_ROOT, ID_CONFIG, "Config", THINGSET_NO_CALLBACK);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG, ID_CONFIG_BLINK_PERIOD, "wBlinkPeriod_s",
                        &blink_period_s, 2, THINGSET_ANY_RW, SUBSET_SER);
