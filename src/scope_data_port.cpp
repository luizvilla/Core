/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "scope_data_port.h"

#include <zephyr/console/console.h>
#include <zephyr/sys/printk.h>

void scope_data_port_task(void)
{
    const int received = console_getchar();

    if (received == '?') {
        printk("SCOPE-DATA/1 OK\n");
    }
    else {
        printk("SCOPE-DATA/1 ERROR UNKNOWN_COMMAND\n");
    }
}
