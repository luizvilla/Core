/*
 * Copyright (c) 2022-present LAAS-CNRS
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

/*
 * @date   2026
 */


/* Current module */
#include "periodic_synchronous_task.h"

#ifdef CONFIG_OWNTECH_TASK_ENABLE_PERIODIC_TASKS

/* OwnTech Power API */
#include "uninterruptible_synchronous_task.h"
#include "scheduling_common.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

/**
 *  Local variables and constants
 */

/**
 * The periodic task reuses TIM7's IRQ vector (TIM7_DAC_IRQn on STM32G4)
 * purely as a software-triggered interrupt line: TIM7 hardware itself is
 * never touched here. This means TIM7 must not be used through the generic
 * timer driver while CONFIG_OWNTECH_TASK_ENABLE_PERIODIC_TASKS is enabled.
 */
static const IRQn_Type PERIODIC_TASK_IRQN = static_cast<IRQn_Type>(55);
#define PERIODIC_TASK_PRIORITY  1

static task_status_t   periodicTaskStatus  = task_status_t::inexistent;
static task_function_t user_periodic_task  = NULL;
static uint32_t         period_ratio       = 0;
static uint32_t         tick_counter       = 0;

/* Private API */

static void periodic_task_proxy()
{
	NVIC_ClearPendingIRQ(PERIODIC_TASK_IRQN);

	if (user_periodic_task != NULL)
	{
		user_periodic_task();
	}
}

/* Public API */

int8_t scheduling_define_periodic_task(task_function_t periodic_task,
										uint32_t task_period_us)
{
	if ( (periodicTaskStatus != task_status_t::inexistent) &&
		 (periodicTaskStatus != task_status_t::suspended) )
		return -1;

	if (periodic_task == NULL)
		return -1;

	uint32_t critical_task_period_us =
					scheduling_get_uninterruptible_synchronous_task_period();

	if (critical_task_period_us == 0)
		return -1;

	if (task_period_us % critical_task_period_us != 0)
		return -1;

	uint32_t ratio = task_period_us / critical_task_period_us;

	if (ratio == 0)
		return -1;

	period_ratio       = ratio;
	tick_counter        = 0;
	user_periodic_task  = periodic_task;

	IRQ_DIRECT_CONNECT(PERIODIC_TASK_IRQN,
						PERIODIC_TASK_PRIORITY,
						periodic_task_proxy,
						IRQ_ZERO_LATENCY);

	periodicTaskStatus = task_status_t::defined;

	return 0;
}

void scheduling_start_periodic_task()
{
	if ( (periodicTaskStatus != task_status_t::defined) &&
		 (periodicTaskStatus != task_status_t::suspended) )
		return;

	tick_counter = 0;

	irq_enable(PERIODIC_TASK_IRQN);

	periodicTaskStatus = task_status_t::running;
}

void scheduling_stop_periodic_task()
{
	if (periodicTaskStatus != task_status_t::running)
		return;

	irq_disable(PERIODIC_TASK_IRQN);

	periodicTaskStatus = task_status_t::suspended;
}

void scheduling_periodic_task_tick()
{
	if (periodicTaskStatus != task_status_t::running)
		return;

	tick_counter++;

	if (tick_counter >= period_ratio)
	{
		tick_counter = 0;
		NVIC_SetPendingIRQ(PERIODIC_TASK_IRQN);
	}
}

#endif /* CONFIG_OWNTECH_TASK_ENABLE_PERIODIC_TASKS */
