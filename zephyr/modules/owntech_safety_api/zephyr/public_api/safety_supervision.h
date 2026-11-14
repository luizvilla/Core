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
 * @file safety_supervision.h
 * @brief CPU supervision (ADR 0009): the independent watchdog, the fatal-error
 *        handler and the HRTIM system fault. Together they bring every PWM
 *        output to its safe state when the CPU itself fails.
 *
 * CPU supervision is always on: it has no user switch. It arms the first time
 * a PWM output is enabled, and from then on:
 *
 * - the independent watchdog (IWDG) resets the MCU if it is not fed for
 *   SAFETY_SUPERVISION_WATCHDOG_TIMEOUT_US. The control task feeds it once per
 *   period; while no control task runs, a kernel timer feeds it instead;
 * - a fatal error (stack overflow, hard fault, k_panic, ...) disables every
 *   HRTIM output before the CPU halts;
 * - a Cortex-M4 lockup, a supply drop (PVD), a clock failure, an SRAM parity
 *   error or a flash ECC error disables every fault-protected HRTIM output in
 *   hardware (HRTIM system fault).
 *
 * A fault caught by CPU supervision is latched across resets: after the
 * reset, PWM outputs stay disabled until safety_supervision_clear_fault() or
 * a power cycle. Nothing restarts the converter on its own.
 */

#ifndef SAFETY_SUPERVISION_H_
#define SAFETY_SUPERVISION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Declared fault reaction time of the watchdog (IEC 60730-1 H.11) when the
 *  control task period is short. The reload keeps the timeout below this
 *  value over the whole LSI range of the STM32G474 (29.5 to 34 kHz). A
 *  control task slower than a third of it lengthens the timeout to at least
 *  3 periods: see safety_supervision_watchdog_timeout_us(). */
#define SAFETY_SUPERVISION_WATCHDOG_TIMEOUT_US 1000U

/** The cause of a latched CPU-supervision fault. */
typedef enum
{
    SAFETY_FAULT_NONE        = 0, /**< No latched fault */
    SAFETY_FAULT_FATAL_ERROR = 1, /**< Zephyr fatal error (stack overflow, hard fault, panic...) */
    SAFETY_FAULT_WATCHDOG    = 2, /**< Independent watchdog reset: the control task stopped */
} safety_fault_t;

/** What CPU supervision recorded about the latched fault. It survives the
 *  reset that follows the fault (TAMP backup registers). */
typedef struct
{
    safety_fault_t fault;     /**< Cause of the fault */
    uint32_t reason;          /**< Zephyr fatal reason (K_ERR_*), for a fatal error */
    uint32_t pc;              /**< Program counter of the fault, 0 if unknown */
    uint32_t disable_cycles;  /**< DWT cycle counter when the outputs were disabled */
    uint32_t outputs_after;   /**< PWM pin levels read right after disabling (bit n = HRTIM output n, TA1..TF2) */
    uint32_t thread;          /**< Current thread (struct k_thread *) at the fault */
    uint32_t context;         /**< IPSR at the fault (active exception number); bit 31 set if in an ISR */
    uint32_t stack;           /**< Current thread stack: start address (bits 31-12) | size / 8 (bits 11-0) */
    uint32_t lr;              /**< Link register of the fault frame: the caller of an assert or panic */
} safety_fault_record_t;

/** @brief Feeds the watchdog. Called by the control task once per period. */
void safety_supervision_feed(void);

/**
 * @brief The control task started: from now on it alone feeds the watchdog.
 * @param period_us control task period; the watchdog timeout becomes at least
 *        3 periods (and never less than the default).
 */
void safety_supervision_feeder_attach(uint32_t period_us);

/** @brief @return the longest watchdog timeout over the LSI range, in µs: the
 *         declared fault reaction time of the watchdog. */
uint32_t safety_supervision_watchdog_timeout_us(void);

/** @brief The control task stopped: the kernel timer feeds the watchdog again. */
void safety_supervision_feeder_detach(void);

/** @brief @return true once CPU supervision is armed (a PWM output was enabled). */
bool safety_supervision_armed(void);

/** @brief @return the latched fault, SAFETY_FAULT_NONE if there is none. */
safety_fault_t safety_supervision_get_fault(void);

/** @brief Copies the record of the latched fault. */
void safety_supervision_get_record(safety_fault_record_t *record);

/**
 * @brief Clears the latched fault so that PWM outputs can be enabled again.
 * @return 0 when cleared; -1 if a PWM output is still enabled (refused).
 */
int safety_supervision_clear_fault(void);

/** @brief Disables every HRTIM output at once (emergency path, any context). */
void safety_outputs_disable_all(void);

/**
 * @brief Test hook: stores a cycle-counter timestamp in the backup registers
 *        just before a fault is provoked, so the reaction time can be read
 *        after the reset. Used only by the safety test firmware.
 */
void safety_supervision_mark(uint32_t cycles);

/** @brief @return the timestamp stored by safety_supervision_mark(). */
uint32_t safety_supervision_get_mark(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_SUPERVISION_H_ */
