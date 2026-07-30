/*
 * Copyright (c) 2026-present LAAS-CNRS
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
 * @brief  Manual/scripted test harness for spin.metaData: write canned
 *         values to flash, reset or reflash the board, then read them
 *         back over serial to verify persistence.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/* --------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>
#include <string.h>

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "nvs_storage.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */
void loop_communication_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */
uint8_t received_serial_char;

/* Canned test values written by the 'w' command and expected back by 'r' */
static const char TEST_SPIN_SERIAL[]     = "SPIN000000001"; /* 13 chars */
static const char TEST_SHIELD_SERIAL[]   = "SHLD000000001"; /* 13 chars */
static const char TEST_SPIN_PASSWORD[]   = "SPINPASS01";    /* 10 chars */
static const char TEST_SHIELD_PASSWORD[] = "SHLDPASS01";    /* 10 chars */
static const uint8_t TEST_SPIN_VERSION[3]   = {9, 9, 9};
static const uint8_t TEST_SHIELD_VERSION[3] = {8, 8, 8};
static const char* TEST_EXTRA[METADATA_EXTRA_COUNT] =
{
    "EXTRA0", "EXTRA1", "EXTRA2", "EXTRA3", "EXTRA4"
};

/* --------------SETUP FUNCTIONS------------------------------- */

void setup_routine()
{
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.startBackground(com_task_number);
}

/* --------------LOOP FUNCTIONS-------------------------------- */

static void print_help()
{
    printk(" ________________________________________________ \n"
           "|     ------- spin.metaData TEST MENU ---------  |\n"
           "|     press h : print this help menu             |\n"
           "|     press w : write canned test values         |\n"
           "|     press b : attempt undersized writes/reads  |\n"
           "|     press r : read back all metadata fields    |\n"
           "|     press c : clear all metadata fields        |\n"
           "|     press f : print free NVS space             |\n"
           "|_________________________________________________|\n\n");
}

static void write_all_fields()
{
    int8_t ret;

    ret = spin.metaData.setSpinSerialNumber(TEST_SPIN_SERIAL,
                                             sizeof(TEST_SPIN_SERIAL) - 1);
    printk("SPIN_SERIAL=%s\n", (ret == 0) ? "OK" : "ERR");

    ret = spin.metaData.setShieldSerialNumber(TEST_SHIELD_SERIAL,
                                               sizeof(TEST_SHIELD_SERIAL) - 1);
    printk("SHIELD_SERIAL=%s\n", (ret == 0) ? "OK" : "ERR");

    ret = spin.metaData.setSpinVersion(TEST_SPIN_VERSION[0],
                                        TEST_SPIN_VERSION[1],
                                        TEST_SPIN_VERSION[2]);
    printk("SPIN_VERSION=%s\n", (ret == 0) ? "OK" : "ERR");

    ret = spin.metaData.setShieldVersion(TEST_SHIELD_VERSION[0],
                                          TEST_SHIELD_VERSION[1],
                                          TEST_SHIELD_VERSION[2]);
    printk("SHIELD_VERSION=%s\n", (ret == 0) ? "OK" : "ERR");

    ret = spin.metaData.setSpinPassword(TEST_SPIN_PASSWORD,
                                         sizeof(TEST_SPIN_PASSWORD) - 1);
    printk("SPIN_PASSWORD=%s\n", (ret == 0) ? "OK" : "ERR");

    ret = spin.metaData.setShieldPassword(TEST_SHIELD_PASSWORD,
                                           sizeof(TEST_SHIELD_PASSWORD) - 1);
    printk("SHIELD_PASSWORD=%s\n", (ret == 0) ? "OK" : "ERR");

    for (uint8_t i = 0 ; i < METADATA_EXTRA_COUNT ; i++)
    {
        ret = spin.metaData.setExtraData(i,
                                          (const uint8_t*)TEST_EXTRA[i],
                                          (uint8_t)strlen(TEST_EXTRA[i]));
        printk("EXTRA_%u=%s\n", i, (ret == 0) ? "OK" : "ERR");
    }

    printk("END_WRITE\n");
}

/**
 * @brief Attempt to write each fixed-length field with fewer bytes than
 *        required, and to read the extra-data slot with a buffer smaller
 *        than METADATA_EXTRA_MAX_LEN, to verify the size checks reject
 *        them (-2) instead of reading/writing past the end of the
 *        caller's buffer.
 */
static void write_undersized_fields()
{
    int8_t ret;

    ret = spin.metaData.setSpinSerialNumber("ABC", 3);
    printk("SPIN_SERIAL_BADSIZE=%d\n", ret);

    ret = spin.metaData.setShieldSerialNumber("XY", 2);
    printk("SHIELD_SERIAL_BADSIZE=%d\n", ret);

    ret = spin.metaData.setSpinPassword("XYZ", 3);
    printk("SPIN_PASSWORD_BADSIZE=%d\n", ret);

    ret = spin.metaData.setShieldPassword("Z", 1);
    printk("SHIELD_PASSWORD_BADSIZE=%d\n", ret);

    uint8_t extra_buf[METADATA_EXTRA_MAX_LEN - 1];
    ret = spin.metaData.getExtraData(0, extra_buf, sizeof(extra_buf));
    printk("EXTRA_BADSIZE=%d\n", ret);

    printk("END_BADSIZE\n");
}

static void read_all_fields()
{
    int8_t ret;

    char serial_buf[SPIN_SERIAL_LEN + 1];
    ret = spin.metaData.getSpinSerialNumber(serial_buf, sizeof(serial_buf));
    if (ret >= 0)
    {
        serial_buf[SPIN_SERIAL_LEN] = '\0';
        printk("SPIN_SERIAL=%s\n", serial_buf);
    }
    else
    {
        printk("SPIN_SERIAL=ERR:%d\n", ret);
    }

    char shield_serial_buf[SHIELD_SERIAL_LEN + 1];
    ret = spin.metaData.getShieldSerialNumber(shield_serial_buf,
                                               sizeof(shield_serial_buf));
    if (ret >= 0)
    {
        shield_serial_buf[SHIELD_SERIAL_LEN] = '\0';
        printk("SHIELD_SERIAL=%s\n", shield_serial_buf);
    }
    else
    {
        printk("SHIELD_SERIAL=ERR:%d\n", ret);
    }

    uint8_t spin_major, spin_minor, spin_rev;
    ret = spin.metaData.getSpinVersion(&spin_major, &spin_minor, &spin_rev);
    if (ret == 0)
    {
        printk("SPIN_VERSION=%u.%u.%u\n", spin_major, spin_minor, spin_rev);
    }
    else
    {
        printk("SPIN_VERSION=ERR:%d\n", ret);
    }

    uint8_t shield_major, shield_minor, shield_rev;
    ret = spin.metaData.getShieldVersion(&shield_major, &shield_minor, &shield_rev);
    if (ret == 0)
    {
        printk("SHIELD_VERSION=%u.%u.%u\n", shield_major, shield_minor, shield_rev);
    }
    else
    {
        printk("SHIELD_VERSION=ERR:%d\n", ret);
    }

    char spin_password_buf[SPIN_PASSWORD_LEN + 1];
    ret = spin.metaData.getSpinPassword(spin_password_buf, sizeof(spin_password_buf));
    if (ret >= 0)
    {
        spin_password_buf[SPIN_PASSWORD_LEN] = '\0';
        printk("SPIN_PASSWORD=%s\n", spin_password_buf);
    }
    else
    {
        printk("SPIN_PASSWORD=ERR:%d\n", ret);
    }

    char password_buf[SHIELD_PASSWORD_LEN + 1];
    ret = spin.metaData.getShieldPassword(password_buf, sizeof(password_buf));
    if (ret >= 0)
    {
        password_buf[SHIELD_PASSWORD_LEN] = '\0';
        printk("SHIELD_PASSWORD=%s\n", password_buf);
    }
    else
    {
        printk("SHIELD_PASSWORD=ERR:%d\n", ret);
    }

    for (uint8_t i = 0 ; i < METADATA_EXTRA_COUNT ; i++)
    {
        char extra_buf[METADATA_EXTRA_MAX_LEN + 1];
        ret = spin.metaData.getExtraData(i, (uint8_t*)extra_buf,
                                          METADATA_EXTRA_MAX_LEN);
        if (ret >= 0)
        {
            extra_buf[ret] = '\0';
            printk("EXTRA_%u=%s\n", i, extra_buf);
        }
        else
        {
            printk("EXTRA_%u=ERR:%d\n", i, ret);
        }
    }

    printk("END_READ\n");
}

void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        print_help();
        break;
    case 'w':
        write_all_fields();
        break;
    case 'b':
        write_undersized_fields();
        break;
    case 'r':
        read_all_fields();
        break;
    case 'c':
    {
        int8_t ret = spin.metaData.clearAllMetaData();
        printk("CLEAR=%s\n", (ret == 0) ? "OK" : "ERR");
        break;
    }
    case 'f':
        printk("FREE_SPACE=%d\n", nvs_storage_get_free_space());
        break;
    default:
        break;
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
