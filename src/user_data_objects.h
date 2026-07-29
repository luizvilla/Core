/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  ThingSet data object definitions for the serial-shell example:
 *         a read-only "Measurements" group and a writable "Config" group.
 */

/* uint32_t, float32_t and other fixed-width integer typedefs. */
#include <stdint.h>
/* memcpy/memset, used internally by some ThingSet item types. */
#include <string.h>

/* Core ThingSet macros: THINGSET_ADD_GROUP, THINGSET_ADD_ITEM_*, etc. */
#include <thingset.h>
/* ThingSet SDK context (thingset_context) used by the shell/CAN transports. */
#include <thingset/sdk.h>

/* Every ThingSet object tree has a root group with ID 0. */
#define ID_ROOT        0x00

/* "Measurements" group ID (child of root) and its items' IDs (children of
 * ID_MEAS). Each ID must be unique among the siblings under the same
 * parent, but groups and items can reuse numbers across different
 * parents. */
#define ID_MEAS        0x5
#define ID_MEAS_V1_LOW 0x50
#define ID_MEAS_V2_LOW 0x51
#define ID_MEAS_V_HIGH 0x52
#define ID_MEAS_I1_LOW 0x53
#define ID_MEAS_I2_LOW 0x54
#define ID_MEAS_I_HIGH 0x55
#define ID_MEAS_TEMP1  0x56
#define ID_MEAS_TEMP2  0x57

/* "Config" group ID (child of root) and its one writable item's ID. */
#define ID_CONFIG              0x6
#define ID_CONFIG_BLINK_PERIOD 0x60

/* Subset bitmask: marks which items get included when a client asks for
 * a named subset of the tree (e.g. periodic reporting). Only one subset
 * is used in this example, so a single bit is enough. */
#define SUBSET_SER (1U << 0)

/* Backing storage for the "Measurements" group. ThingSet items only ever
 * store a pointer to these variables, so main.cpp updates them directly
 * (e.g. from sensor readings) and the new value is what GET returns. */
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high_value;
static float32_t V_high_value;
static float32_t temp_1_value;
static float32_t temp_2_value;
/* Scratch variable main.cpp reuses to fetch each sensor reading before
 * copying it into the corresponding *_value above. */
static float32_t meas_data;

/* Writable over the ThingSet shell: LED blink half-period, in seconds. */
static float32_t blink_period_s = 1.0f;

/* Register the "Measurements" group as a child of the root (ID_ROOT),
 * with its own ID (ID_MEAS) and display name. THINGSET_NO_CALLBACK means
 * no function runs before/after this group is accessed. */
THINGSET_ADD_GROUP(ID_ROOT, ID_MEAS, "Measurements", THINGSET_NO_CALLBACK);

/* Each THINGSET_ADD_ITEM_FLOAT below takes: parent group ID, item ID,
 * ThingSet name, pointer to the backing variable, number of decimal
 * places to show, access mode, and subset bitmask. Names follow the
 * ThingSet convention "r<Name>_<Unit>": leading "r" marks it read-only,
 * the suffix after "_" is the physical unit. THINGSET_ANY_R allows GET
 * from any authentication level (there's no write access here). */

/* Low-side voltage on leg 1, in volts. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V1_LOW, "rV1Low_V", &V1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* Low-side voltage on leg 2, in volts. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V2_LOW, "rV2Low_V", &V2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* High-side (bus) voltage, in volts. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V_HIGH, "rVHigh_V", &V_high_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* Low-side current on leg 1, in amps. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I1_LOW, "rI1Low_A", &I1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* Low-side current on leg 2, in amps. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I2_LOW, "rI2Low_A", &I2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* High-side (bus) current, in amps. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I_HIGH, "rIHigh_A", &I_high_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* Temperature sensor 1, in degrees Celsius. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP1, "rTemp1_degC", &temp_1_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);
/* Temperature sensor 2, in degrees Celsius. */
THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP2, "rTemp2_degC", &temp_2_value, 2,
                        THINGSET_ANY_R, SUBSET_SER);

/* Register the "Config" group as a child of the root. */
THINGSET_ADD_GROUP(ID_ROOT, ID_CONFIG, "Config", THINGSET_NO_CALLBACK);

/* LED blink half-period, in seconds. The leading "w" marks it writable
 * (not persisted to flash); THINGSET_ANY_RW allows both GET and UPDATE
 * from any authentication level. main.cpp reads this variable directly
 * to time the LED toggle, so writing it over the shell changes the
 * blink rate immediately. */
THINGSET_ADD_ITEM_FLOAT(ID_CONFIG, ID_CONFIG_BLINK_PERIOD, "wBlinkPeriod_s",
                        &blink_period_s, 2, THINGSET_ANY_RW, SUBSET_SER);
