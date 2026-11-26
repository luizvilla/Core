/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "user_data_objects.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "ShieldAPI.h"
#include "SpinAPI.h"

/* converter_metadata's serial_number is a null-terminated mirror of
 * spin.metaData's fixed-length, non-null-terminated SHIELD_SERIAL_LEN
 * buffer; keep the two in sync. */
static_assert(CONVERTER_SERIAL_NUMBER_SIZE == SHIELD_SERIAL_LEN + 1,
              "converter_metadata_t.serial_number size must match "
              "MetaDataAPI's SHIELD_SERIAL_LEN");

static converter_metadata_t previous_converter_metadata;

static bool raw_bytes_are_printable_ascii(const char *value, size_t length)
{
    if (length == 0) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        const unsigned char character = static_cast<unsigned char>(value[i]);
        if (character < 0x20 || character > 0x7E) {
            return false;
        }
    }
    return true;
}

/* Parses a "[v]MAJOR.MINOR.REV" string (the optional leading 'v'/'V' matches
 * the devicetree shield-version convention, e.g. "v1.4.2") into three
 * uint8_t components suitable for spin.metaData.setShieldVersion().
 *
 * Hand-rolled instead of sscanf(): on this toolchain, even the "integer-only"
 * scanf variants route through iswspace()/iswspace_l(), pulling in ~27 KB of
 * Unicode category/JIS-conversion tables (libc_a-categories.o,
 * libc_a-jp2uc.o) that this firmware has no other reason to need. strtoul()
 * has no such dependency (only _ctype_/_impure_ptr). */
static bool parse_version_string(const char *value, uint8_t &major,
                                  uint8_t &minor, uint8_t &rev)
{
    if (*value == 'v' || *value == 'V') {
        ++value;
    }

    unsigned long components[3];
    for (int i = 0; i < 3; ++i) {
        if (*value < '0' || *value > '9') {
            return false;
        }

        char *end = nullptr;
        components[i] = strtoul(value, &end, 10);
        if (components[i] > UINT8_MAX) {
            return false;
        }

        value = end;
        if (i < 2) {
            if (*value != '.') {
                return false;
            }
            ++value;
        }
    }

    if (*value != '\0') {
        return false;
    }

    major = static_cast<uint8_t>(components[0]);
    minor = static_cast<uint8_t>(components[1]);
    rev   = static_cast<uint8_t>(components[2]);
    return true;
}

/* Formats "major.minor.rev" without going through snprintf(): on this
 * toolchain, snprintf() unconditionally routes through the float-capable
 * _svfprintf_r backend (~7 KB) regardless of format string content, and
 * even sniprintf() (the "integer-only" variant) still pulls in wide-char
 * conversion support. major/minor/rev are uint8_t, so the result never
 * exceeds "255.255.255" (11 chars) plus a null terminator. */
static void format_version_string(char *buffer, size_t buffer_size,
                                   uint8_t major, uint8_t minor, uint8_t rev)
{
    const uint8_t components[3] = {major, minor, rev};
    size_t pos = 0;

    for (int i = 0; i < 3; ++i) {
        char digits[3];
        size_t digit_count = 0;
        uint8_t value = components[i];

        do {
            digits[digit_count++] = static_cast<char>('0' + (value % 10));
            value /= 10;
        } while (value != 0);

        while (digit_count > 0 && pos < buffer_size - 1) {
            buffer[pos++] = digits[--digit_count];
        }

        if (i < 2 && pos < buffer_size - 1) {
            buffer[pos++] = '.';
        }
    }

    buffer[pos] = '\0';
}

void load_converter_metadata(void)
{
    char serial_buffer[SHIELD_SERIAL_LEN];
    if (spin.metaData.getShieldSerialNumber(serial_buffer,
                                             sizeof(serial_buffer)) >= 0) {
        memcpy(converter_metadata.serial_number, serial_buffer,
               sizeof(serial_buffer));
        converter_metadata.serial_number[SHIELD_SERIAL_LEN] = '\0';
    }

    uint8_t major, minor, rev;
    if (spin.metaData.getShieldVersion(&major, &minor, &rev) == 0) {
        format_version_string(converter_metadata.board_version,
                               sizeof(converter_metadata.board_version),
                               major, minor, rev);
    }
}

void converter_metadata_cb(enum thingset_callback_reason reason)
{
    switch (reason) {
        case THINGSET_CALLBACK_PRE_WRITE:
            previous_converter_metadata = converter_metadata;
            break;

        case THINGSET_CALLBACK_POST_WRITE: {
            if (strncmp(converter_metadata.serial_number,
                        previous_converter_metadata.serial_number,
                        sizeof(converter_metadata.serial_number)) != 0) {
                const size_t length = strnlen(
                    converter_metadata.serial_number,
                    sizeof(converter_metadata.serial_number));

                if (length != SHIELD_SERIAL_LEN ||
                    !raw_bytes_are_printable_ascii(
                        converter_metadata.serial_number, length) ||
                    spin.metaData.setShieldSerialNumber(
                        converter_metadata.serial_number,
                        static_cast<uint8_t>(length)) != 0) {
                    memcpy(converter_metadata.serial_number,
                           previous_converter_metadata.serial_number,
                           sizeof(converter_metadata.serial_number));
                }
            }

            if (strncmp(converter_metadata.board_version,
                        previous_converter_metadata.board_version,
                        sizeof(converter_metadata.board_version)) != 0) {
                uint8_t major, minor, rev;

                if (!parse_version_string(converter_metadata.board_version,
                                          major, minor, rev) ||
                    spin.metaData.setShieldVersion(major, minor, rev) != 0) {
                    memcpy(converter_metadata.board_version,
                           previous_converter_metadata.board_version,
                           sizeof(converter_metadata.board_version));
                }
            }
            break;
        }

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
                /* There is no working upper bound to validate against: HRTIM
                 * never populates pwm_conf.max_frequency (spin.pwm.getFrequencyMax()
                 * always returns 0), and neither PowerAPI nor the devicetree
                 * power-leg binding define a max-frequency concept. Only the
                 * lower bound is enforced until a real ceiling is modeled. */
                const uint32_t min_frequency = shield.power.getFrequencyMin();

                if (switching_frequency_hz < min_frequency) {
                    switching_frequency_hz = previous_frequency_hz;
                }
                else {
                    shield.power.setFrequency(switching_frequency_hz);
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
