/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#ifndef USER_DATA_OBJECTS_H_
#define USER_DATA_OBJECTS_H_

/* ThingSet object IDs. Existing IDs are kept stable for client compatibility. */
#define ID_ROOT 0x00

#define ID_MEAS        0x05
#define ID_MEAS_V1_LOW 0x50
#define ID_MEAS_V2_LOW 0x51
#define ID_MEAS_V_HIGH 0x52
#define ID_MEAS_I1_LOW 0x53
#define ID_MEAS_I2_LOW 0x54
#define ID_MEAS_I_HIGH 0x55
#define ID_MEAS_TEMP1  0x56
#define ID_MEAS_TEMP2  0x57

#define ID_CONFIG              0x06
#define ID_CONFIG_BLINK_PERIOD 0x60

#define SUBSET_SER (1U << 0)

extern float V1_low_value;
extern float V2_low_value;
extern float V_high_value;
extern float I1_low_value;
extern float I2_low_value;
extern float I_high_value;
extern float temp_1_value;
extern float temp_2_value;
extern float meas_data;

extern float blink_period_s;

#endif /* USER_DATA_OBJECTS_H_ */
