/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "scope_data_port.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/console/console.h>
#include <zephyr/sys/printk.h>

#include "scope_capture.h"

static void print_capture(void)
{
    if (!scope_capture_begin_stream()) {
        printk("SCOPE-DATA/1 ERROR NOT_READY %s\n",
               scope_capture_state_name());
        return;
    }

    printk("begin record\n#");
    for (uint16_t channel = 0; channel < SCOPE_CHANNEL_COUNT; ++channel) {
        printk("%s,", scope_capture_channel_name(channel));
    }
    printk("\n# %u\n", scope_final_index);

    const uint8_t *buffer = scope_capture_buffer();
    const uint16_t buffer_size = scope_capture_buffer_size();
    for (uint16_t offset = 0; offset < buffer_size; offset += sizeof(uint32_t)) {
        uint32_t encoded_value;
        memcpy(&encoded_value, buffer + offset, sizeof(encoded_value));
        printk("%08x\n", static_cast<unsigned int>(encoded_value));
    }
    printk("end record\n");

    scope_capture_end_stream(true);
}

void scope_data_port_task(void)
{
    const int received = console_getchar();

    if (received == '?') {
        printk("SCOPE-DATA/1 OK\n");
    }
    else if (received == 'D') {
        print_capture();
    }
    else {
        printk("SCOPE-DATA/1 ERROR UNKNOWN_COMMAND\n");
    }
}
