/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

#include <stddef.h>
#include <string.h>

#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "nvs_storage.h"

static constexpr uint16_t CONVERTER_METADATA_NVS_ID = 0x0401;
static converter_metadata_t previous_converter_metadata;

static_assert(sizeof(converter_metadata_t) == 120);
static_assert(sizeof(converter_metadata_t) <= 127,
              "NVS storage API returns the byte count as int8_t");

static bool metadata_string_is_valid(const char *value, size_t capacity)
{
    size_t length = 0;
    while (length < capacity && value[length] != '\0') {
        const unsigned char character = static_cast<unsigned char>(value[length]);
        if (character < 0x20 || character > 0x7E) {
            return false;
        }
        ++length;
    }
    return length > 0 && length < capacity;
}

static bool converter_metadata_is_valid(const converter_metadata_t &metadata)
{
    return metadata_string_is_valid(metadata.board_name,
                                    sizeof(metadata.board_name)) &&
           metadata_string_is_valid(metadata.board_version,
                                    sizeof(metadata.board_version)) &&
           metadata_string_is_valid(metadata.serial_number,
                                    sizeof(metadata.serial_number)) &&
           metadata_string_is_valid(metadata.firmware_version,
                                    sizeof(metadata.firmware_version));
}

void load_converter_metadata(void)
{
    converter_metadata_t stored_metadata;
    const int read_size = nvs_storage_retrieve_data(
        CONVERTER_METADATA_NVS_ID, &stored_metadata, sizeof(stored_metadata));

    if (read_size == sizeof(stored_metadata) &&
        converter_metadata_is_valid(stored_metadata)) {
        converter_metadata = stored_metadata;
    }
}

void converter_metadata_cb(enum thingset_callback_reason reason)
{
    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous_converter_metadata = converter_metadata;
            break;

        case THINGSET_CALLBACK_POST_WRITE:
            if (memcmp(&converter_metadata, &previous_converter_metadata,
                       sizeof(converter_metadata)) == 0) {
                break;
            }

            if (!converter_metadata_is_valid(converter_metadata) ||
                nvs_storage_store_data(CONVERTER_METADATA_NVS_ID,
                                       &converter_metadata,
                                       sizeof(converter_metadata)) < 0) {
                converter_metadata = previous_converter_metadata;
            }
            break;

        default:
            break;
    }
}

static uint8_t previous_mode;
static uint32_t previous_frequency_hz;

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

void conf_freq_cb(enum thingset_callback_reason reason)
{
    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous_frequency_hz = switching_frequency_hz;
            break;

        case THINGSET_CALLBACK_POST_WRITE:
            if (switching_frequency_hz != previous_frequency_hz) {
                const uint32_t min_frequency = spin.pwm.getFrequencyMin(PWMA);
                const uint32_t max_frequency = spin.pwm.getFrequencyMax(PWMA);

                if (switching_frequency_hz < min_frequency ||
                    switching_frequency_hz > max_frequency) {
                    switching_frequency_hz = previous_frequency_hz;
                }
                else {
                    spin.pwm.setFrequency(switching_frequency_hz);
                }
            }
            break;

        default:
            break;
    }
}

void conf_config_cb(enum thingset_callback_reason reason)
{
    conf_mode_cb(reason);
    conf_freq_cb(reason);
}

typedef struct
{
    float duty_cycle;
    char tracking_name[TRACKING_NAME_SIZE];
    bool capacitor;
    bool driver;
    int16_t phase_shift;
    uint16_t dead_time_rising_ns;
    uint16_t dead_time_falling_ns;
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
            previous.capacitor = leg.capacitor;
            previous.driver = leg.driver;
            previous.phase_shift = leg.phase_shift;
            previous.dead_time_rising_ns = leg.dead_time_rising_ns;
            previous.dead_time_falling_ns = leg.dead_time_falling_ns;
            break;

        case THINGSET_CALLBACK_POST_WRITE: {
            const leg_t hardware_leg = leg_index == 0 ? LEG1 : LEG2;

            if (leg.duty_cycle < 0.0f || leg.duty_cycle > 1.0f) {
                leg.duty_cycle = previous.duty_cycle;
            }

            if (leg.capacitor != previous.capacitor) {
                if (leg.capacitor) {
                    shield.power.connectCapacitor(hardware_leg);
                }
                else {
                    shield.power.disconnectCapacitor(hardware_leg);
                }
            }

            if (leg.driver != previous.driver) {
                if (leg.driver) {
                    shield.power.connectDriver(hardware_leg);
                }
                else {
                    shield.power.disconnectDriver(hardware_leg);
                }
            }

            if (leg.phase_shift != previous.phase_shift) {
                if (leg.phase_shift < -360 || leg.phase_shift > 360) {
                    leg.phase_shift = previous.phase_shift;
                }
                else {
                    shield.power.setPhaseShift(hardware_leg, leg.phase_shift);
                }
            }

            if (leg.dead_time_rising_ns != previous.dead_time_rising_ns ||
                leg.dead_time_falling_ns != previous.dead_time_falling_ns) {
                shield.power.setDeadTime(hardware_leg, leg.dead_time_rising_ns,
                                         leg.dead_time_falling_ns);
            }

            if (strncmp(leg.tracking_name, previous.tracking_name,
                        TRACKING_NAME_SIZE) != 0) {
                bool found = false;
                for (uint8_t i = 0; i < CALIBRATION_CHANNEL_COUNT; ++i) {
                    if (strcmp(leg.tracking_name, calibration_channels[i].name) == 0) {
                        leg.tracking_var = calibration_channels[i].address;
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    memcpy(leg.tracking_name, previous.tracking_name,
                           TRACKING_NAME_SIZE);
                }
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

typedef struct
{
    float gain;
    float offset;
    bool store;
} calibration_shadow_t;

static calibration_shadow_t previous_calibration[CALIBRATION_CHANNEL_COUNT];

static void cal_channel_cb(enum thingset_callback_reason reason, uint8_t index)
{
    calibration_channel_t &channel = calibration_channels[index];
    calibration_shadow_t &previous = previous_calibration[index];

    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous.gain = channel.gain;
            previous.offset = channel.offset;
            previous.store = channel.store;
            break;

        case THINGSET_CALLBACK_POST_WRITE: {
            const sensor_t sensor = static_cast<sensor_t>(channel.sensor);

            if (channel.gain != previous.gain || channel.offset != previous.offset) {
                shield.sensors.setConversionParametersLinear(
                    sensor, channel.gain, channel.offset);
            }

            if (channel.store && !previous.store) {
                shield.sensors.storeParametersInMemory(sensor);
                shield.sensors.retrieveParametersFromMemory(sensor);
                channel.store = false;
            }
            break;
        }

        default:
            break;
    }
}

void cal_channel_cb_0(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 0);
}

void cal_channel_cb_1(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 1);
}

void cal_channel_cb_2(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 2);
}

void cal_channel_cb_3(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 3);
}

void cal_channel_cb_4(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 4);
}

void cal_channel_cb_5(enum thingset_callback_reason reason)
{
    cal_channel_cb(reason, 5);
}
