/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#ifndef USER_DATA_OBJECTS_H_
#define USER_DATA_OBJECTS_H_

#include <stdbool.h>
#include <stdint.h>

#include <thingset.h>

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
#define ID_CONFIG_MODE         0x61
#define ID_CONFIG_LEG1         0x70
#define ID_CONFIG_LEG1_ENABLE  0x701
#define ID_CONFIG_LEG2         0x80
#define ID_CONFIG_LEG2_ENABLE  0x801

#define SUBSET_SER (1U << 0)
#define POWER_LEG_COUNT 2

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

typedef struct
{
    const char *name;
    uint8_t value;
} ModeDef;

typedef enum : uint8_t
{
    IDLE = 0,
    POWER_OFF,
    POWER_ON,
    NUM_OF_MODES
} tester_state_t;

typedef struct
{
    bool enable;
    bool running;
} power_leg_t;

extern const ModeDef modes[NUM_OF_MODES];
extern uint8_t mode;
extern power_leg_t power_legs[POWER_LEG_COUNT];

void conf_mode_cb(enum thingset_callback_reason reason);

#endif /* USER_DATA_OBJECTS_H_ */
