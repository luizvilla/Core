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
#define ID_MEAS_V1_MAX 0x58
#define ID_MEAS_V2_MAX 0x59
#define ID_MEAS_DUTY1  0x5A
#define ID_MEAS_DUTY2  0x5B

#define ID_CONFIG              0x06
#define ID_CONFIG_BLINK_PERIOD 0x60
#define ID_CONFIG_MODE         0x61
#define ID_CONFIG_FREQ         0x62
#define ID_CONFIG_LEG1         0x70
#define ID_CONFIG_LEG1_ENABLE  0x701
#define ID_CONFIG_LEG1_BUCK    0x702
#define ID_CONFIG_LEG1_BOOST   0x703
#define ID_CONFIG_LEG1_DUTY    0x704
#define ID_CONFIG_LEG1_REF     0x705
#define ID_CONFIG_LEG1_TRACK   0x706
#define ID_CONFIG_LEG1_CAPA    0x707
#define ID_CONFIG_LEG1_DRIVER  0x708
#define ID_CONFIG_LEG1_PHASE   0x709
#define ID_CONFIG_LEG1_DT_RISE 0x70A
#define ID_CONFIG_LEG1_DT_FALL 0x70B
#define ID_CONFIG_LEG2         0x80
#define ID_CONFIG_LEG2_ENABLE  0x801
#define ID_CONFIG_LEG2_BUCK    0x802
#define ID_CONFIG_LEG2_BOOST   0x803
#define ID_CONFIG_LEG2_DUTY    0x804
#define ID_CONFIG_LEG2_REF     0x805
#define ID_CONFIG_LEG2_TRACK   0x806
#define ID_CONFIG_LEG2_CAPA    0x807
#define ID_CONFIG_LEG2_DRIVER  0x808
#define ID_CONFIG_LEG2_PHASE   0x809
#define ID_CONFIG_LEG2_DT_RISE 0x80A
#define ID_CONFIG_LEG2_DT_FALL 0x80B

#define ID_CONVERTER                  0x07
#define ID_CONVERTER_BOARD_NAME       0x71
#define ID_CONVERTER_BOARD_VERSION    0x72
#define ID_CONVERTER_SERIAL_NUMBER    0x73
#define ID_CONVERTER_FIRMWARE_VERSION 0x74

#define SUBSET_SER (1U << 0)
#define POWER_LEG_COUNT 2
#define TRACKING_NAME_SIZE 8
#define CALIBRATION_CHANNEL_COUNT 6
#define CONVERTER_BOARD_NAME_SIZE 24
#define CONVERTER_BOARD_VERSION_SIZE 16
#define CONVERTER_SERIAL_NUMBER_SIZE 48
#define CONVERTER_FIRMWARE_VERSION_SIZE 32

#define ID_CALIBRATION 0x09

#define ID_CAL_V1        0x90
#define ID_CAL_V1_GAIN   0x901
#define ID_CAL_V1_OFFSET 0x902
#define ID_CAL_V1_STORE  0x903

#define ID_CAL_V2        0x91
#define ID_CAL_V2_GAIN   0x911
#define ID_CAL_V2_OFFSET 0x912
#define ID_CAL_V2_STORE  0x913

#define ID_CAL_VH        0x92
#define ID_CAL_VH_GAIN   0x921
#define ID_CAL_VH_OFFSET 0x922
#define ID_CAL_VH_STORE  0x923

#define ID_CAL_I1        0x93
#define ID_CAL_I1_GAIN   0x931
#define ID_CAL_I1_OFFSET 0x932
#define ID_CAL_I1_STORE  0x933

#define ID_CAL_I2        0x94
#define ID_CAL_I2_GAIN   0x941
#define ID_CAL_I2_OFFSET 0x942
#define ID_CAL_I2_STORE  0x943

#define ID_CAL_IH        0x95
#define ID_CAL_IH_GAIN   0x951
#define ID_CAL_IH_OFFSET 0x952
#define ID_CAL_IH_STORE  0x953

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
extern uint32_t switching_frequency_hz;

typedef struct
{
    const char *name;
    uint8_t value;
} ModeDef;

typedef enum : uint8_t
{
    IDLE = 0,
    POWER_ON,
    POWER_OFF,
    NUM_OF_MODES
} tester_state_t;

typedef struct
{
    bool enable;
    bool running;
    bool buck;
    bool boost;
    float duty_cycle;
    float reference_value;
    float *tracking_var;
    char tracking_name[TRACKING_NAME_SIZE];
    bool capacitor;
    bool driver;
    int16_t phase_shift;
    uint16_t dead_time_rising_ns;
    uint16_t dead_time_falling_ns;
    float v_max;
    float duty_readback;
} power_leg_t;

typedef struct
{
    const char *name;
    float *address;
    uint8_t sensor;
    float gain;
    float offset;
    bool store;
} calibration_channel_t;

typedef struct
{
    char board_name[CONVERTER_BOARD_NAME_SIZE];
    char board_version[CONVERTER_BOARD_VERSION_SIZE];
    char serial_number[CONVERTER_SERIAL_NUMBER_SIZE];
    char firmware_version[CONVERTER_FIRMWARE_VERSION_SIZE];
} converter_metadata_t;

extern const ModeDef modes[NUM_OF_MODES];
extern uint8_t mode;
extern power_leg_t power_legs[POWER_LEG_COUNT];
extern calibration_channel_t calibration_channels[CALIBRATION_CHANNEL_COUNT];
extern converter_metadata_t converter_metadata;

void load_converter_metadata(void);
void converter_metadata_cb(enum thingset_callback_reason reason);
void conf_mode_cb(enum thingset_callback_reason reason);
void conf_freq_cb(enum thingset_callback_reason reason);
void conf_config_cb(enum thingset_callback_reason reason);
void conf_leg_cb_0(enum thingset_callback_reason reason);
void conf_leg_cb_1(enum thingset_callback_reason reason);
void cal_channel_cb_0(enum thingset_callback_reason reason);
void cal_channel_cb_1(enum thingset_callback_reason reason);
void cal_channel_cb_2(enum thingset_callback_reason reason);
void cal_channel_cb_3(enum thingset_callback_reason reason);
void cal_channel_cb_4(enum thingset_callback_reason reason);
void cal_channel_cb_5(enum thingset_callback_reason reason);

#endif /* USER_DATA_OBJECTS_H_ */
