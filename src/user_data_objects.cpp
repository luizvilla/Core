/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

#include <thingset.h>
#include <thingset/sdk.h>

#include "ShieldAPI.h"

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
uint32_t switching_frequency_hz = 200000;

const ModeDef modes[NUM_OF_MODES] = {
    { "IDLE", IDLE },
    { "POWER_ON", POWER_ON },
    { "POWER_OFF", POWER_OFF },
};

uint8_t mode = IDLE;

calibration_channel_t calibration_channels[CALIBRATION_CHANNEL_COUNT] = {
    { "V1", &V1_low_value, V1_LOW, 1.0f, 0.0f, false },
    { "V2", &V2_low_value, V2_LOW, 1.0f, 0.0f, false },
    { "VH", &V_high_value, V_HIGH, 1.0f, 0.0f, false },
    { "I1", &I1_low_value, I1_LOW, 1.0f, 0.0f, false },
    { "I2", &I2_low_value, I2_LOW, 1.0f, 0.0f, false },
    { "IH", &I_high_value, I_HIGH, 1.0f, 0.0f, false },
};

power_leg_t power_legs[POWER_LEG_COUNT] = {
    { false, false, false, false, 0.1f, 0.0f, &V1_low_value, "V1",
      false, false, 0, 100, 100, 0.0f, 0.0f },
    { false, false, false, false, 0.1f, 0.0f, &V2_low_value, "V2",
      false, false, 0, 100, 100, 0.0f, 0.0f },
};

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
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V1_MAX, "rV1Max_V",
                        &power_legs[0].v_max, 2, THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V2_MAX, "rV2Max_V",
                        &power_legs[1].v_max, 2, THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_DUTY1, "rDuty1",
                        &power_legs[0].duty_readback, 3,
                        THINGSET_ANY_R, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_DUTY2, "rDuty2",
                        &power_legs[1].duty_readback, 3,
                        THINGSET_ANY_R, SUBSET_SER);

THINGSET_ADD_GROUP(ID_ROOT, ID_CONFIG, "Config", &conf_config_cb);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG, ID_CONFIG_BLINK_PERIOD, "wBlinkPeriod_s",
                        &blink_period_s, 2, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT8(ID_CONFIG, ID_CONFIG_MODE, "Mode", &mode,
                        THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT32(ID_CONFIG, ID_CONFIG_FREQ, "Frequency_Hz",
                         &switching_frequency_hz, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CONFIG, ID_CONFIG_LEG1, "Leg1", &conf_leg_cb_0);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG1, ID_CONFIG_LEG1_ENABLE, "wEnable",
                       &power_legs[0].enable, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG1, ID_CONFIG_LEG1_BUCK, "wBuck",
                       &power_legs[0].buck, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG1, ID_CONFIG_LEG1_BOOST, "wBoost",
                       &power_legs[0].boost, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG_LEG1, ID_CONFIG_LEG1_DUTY, "wDutyCycle",
                        &power_legs[0].duty_cycle, 3, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG_LEG1, ID_CONFIG_LEG1_REF, "wReferenceValue",
                        &power_legs[0].reference_value, 3, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_STRING(ID_CONFIG_LEG1, ID_CONFIG_LEG1_TRACK, "wTrackingVar",
                         power_legs[0].tracking_name, TRACKING_NAME_SIZE,
                         THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG1, ID_CONFIG_LEG1_CAPA, "wCapa",
                       &power_legs[0].capacitor, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG1, ID_CONFIG_LEG1_DRIVER, "wDriver",
                       &power_legs[0].driver, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_INT16(ID_CONFIG_LEG1, ID_CONFIG_LEG1_PHASE, "wPhaseShift",
                        &power_legs[0].phase_shift, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT16(ID_CONFIG_LEG1, ID_CONFIG_LEG1_DT_RISE,
                         "wDeadTimeRising_ns", &power_legs[0].dead_time_rising_ns,
                         THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT16(ID_CONFIG_LEG1, ID_CONFIG_LEG1_DT_FALL,
                         "wDeadTimeFalling_ns", &power_legs[0].dead_time_falling_ns,
                         THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CONFIG, ID_CONFIG_LEG2, "Leg2", &conf_leg_cb_1);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG2, ID_CONFIG_LEG2_ENABLE, "wEnable",
                       &power_legs[1].enable, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG2, ID_CONFIG_LEG2_BUCK, "wBuck",
                       &power_legs[1].buck, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG2, ID_CONFIG_LEG2_BOOST, "wBoost",
                       &power_legs[1].boost, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG_LEG2, ID_CONFIG_LEG2_DUTY, "wDutyCycle",
                        &power_legs[1].duty_cycle, 3, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG_LEG2, ID_CONFIG_LEG2_REF, "wReferenceValue",
                        &power_legs[1].reference_value, 3, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_STRING(ID_CONFIG_LEG2, ID_CONFIG_LEG2_TRACK, "wTrackingVar",
                         power_legs[1].tracking_name, TRACKING_NAME_SIZE,
                         THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG2, ID_CONFIG_LEG2_CAPA, "wCapa",
                       &power_legs[1].capacitor, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CONFIG_LEG2, ID_CONFIG_LEG2_DRIVER, "wDriver",
                       &power_legs[1].driver, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_INT16(ID_CONFIG_LEG2, ID_CONFIG_LEG2_PHASE, "wPhaseShift",
                        &power_legs[1].phase_shift, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT16(ID_CONFIG_LEG2, ID_CONFIG_LEG2_DT_RISE,
                         "wDeadTimeRising_ns", &power_legs[1].dead_time_rising_ns,
                         THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_UINT16(ID_CONFIG_LEG2, ID_CONFIG_LEG2_DT_FALL,
                         "wDeadTimeFalling_ns", &power_legs[1].dead_time_falling_ns,
                         THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_ROOT, ID_CALIBRATION, "Calibration", THINGSET_NO_CALLBACK);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_V1, "V1", &cal_channel_cb_0);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_V1, ID_CAL_V1_GAIN, "wGain",
                        &calibration_channels[0].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_V1, ID_CAL_V1_OFFSET, "wOffset",
                        &calibration_channels[0].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_V1, ID_CAL_V1_STORE, "wStore",
                       &calibration_channels[0].store, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_V2, "V2", &cal_channel_cb_1);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_V2, ID_CAL_V2_GAIN, "wGain",
                        &calibration_channels[1].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_V2, ID_CAL_V2_OFFSET, "wOffset",
                        &calibration_channels[1].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_V2, ID_CAL_V2_STORE, "wStore",
                       &calibration_channels[1].store, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_VH, "VH", &cal_channel_cb_2);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_VH, ID_CAL_VH_GAIN, "wGain",
                        &calibration_channels[2].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_VH, ID_CAL_VH_OFFSET, "wOffset",
                        &calibration_channels[2].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_VH, ID_CAL_VH_STORE, "wStore",
                       &calibration_channels[2].store, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_I1, "I1", &cal_channel_cb_3);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_I1, ID_CAL_I1_GAIN, "wGain",
                        &calibration_channels[3].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_I1, ID_CAL_I1_OFFSET, "wOffset",
                        &calibration_channels[3].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_I1, ID_CAL_I1_STORE, "wStore",
                       &calibration_channels[3].store, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_I2, "I2", &cal_channel_cb_4);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_I2, ID_CAL_I2_GAIN, "wGain",
                        &calibration_channels[4].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_I2, ID_CAL_I2_OFFSET, "wOffset",
                        &calibration_channels[4].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_I2, ID_CAL_I2_STORE, "wStore",
                       &calibration_channels[4].store, THINGSET_ANY_RW, SUBSET_SER);

THINGSET_ADD_GROUP(ID_CALIBRATION, ID_CAL_IH, "IH", &cal_channel_cb_5);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_IH, ID_CAL_IH_GAIN, "wGain",
                        &calibration_channels[5].gain, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_FLOAT(ID_CAL_IH, ID_CAL_IH_OFFSET, "wOffset",
                        &calibration_channels[5].offset, 6, THINGSET_ANY_RW, SUBSET_SER);
THINGSET_ADD_ITEM_BOOL(ID_CAL_IH, ID_CAL_IH_STORE, "wStore",
                       &calibration_channels[5].store, THINGSET_ANY_RW, SUBSET_SER);
