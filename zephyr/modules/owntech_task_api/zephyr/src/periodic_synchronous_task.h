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


#ifndef PERIODICSYNCHRONOUSTASK_H_
#define PERIODICSYNCHRONOUSTASK_H_

/* Stdlib */
#include <stdint.h>

/* OwnTech Power API */
#include "TaskAPI.h"


#ifdef CONFIG_OWNTECH_TASK_ENABLE_PERIODIC_TASKS

/**
 * @brief Define the periodic task.
 *
 * Registers a hard real-time task that runs at a period which must be a
 * positive integer multiple of the critical task's period. The task is
 * triggered by a software-only Zero Latency Interrupt, one priority level
 * below the critical task's, so the critical task can preempt it while it
 * still preempts regular Zephyr threads without RTOS scheduling jitter.
 *
 * The critical task (see uninterruptible_synchronous_task.h) must already
 * be defined before calling this function, since its period is the base
 * unit the periodic task's period is checked against.
 *
 * @param periodic_task Pointer to the task function (must not be `NULL`).
 * @param task_period_us Task period in microseconds. Must be a positive
 *        integer multiple of the critical task's period.
 *
 * @return `0` on success, `-1` on failure (critical task not defined yet,
 *         invalid period, or periodic task already defined).
 */
int8_t scheduling_define_periodic_task(task_function_t periodic_task,
                                        uint32_t task_period_us);

/**
 * @brief Start the previously defined periodic task.
 */
void scheduling_start_periodic_task();

/**
 * @brief Stop the currently running periodic task.
 *
 * The task can later be resumed by calling scheduling_start_periodic_task()
 * again.
 */
void scheduling_stop_periodic_task();

/**
 * @brief Advance the periodic task's internal counter by one critical-task
 *        tick, triggering the periodic task's interrupt when due.
 *
 * Meant to be called from the critical task's proxy on every one of its
 * ticks, regardless of whether the critical task is sourced from `TIM6` or
 * `HRTIM`.
 */
void scheduling_periodic_task_tick();

#endif /* CONFIG_OWNTECH_TASK_ENABLE_PERIODIC_TASKS */

#endif /* PERIODICSYNCHRONOUSTASK_H_ */
