/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_timing.h
 * @brief Cycle-counter probes of the control interrupt (CONFIG_OWNTECH_SAFETY_TIMING_PROBES,
 *        timing tests only; issue #27, docs/safety/timing_overhead.md).
 */

#ifndef SAFETY_TIMING_H_
#define SAFETY_TIMING_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Blocks of the control interrupt, in execution order (user_task_proxy). */
enum {
    SAFETY_TB_PRE = 0,      /**< old software check (safety_task) */
    SAFETY_TB_DISPATCH,     /**< data dispatch */
    SAFETY_TB_USER,         /**< the application's control task */
    SAFETY_TB_READBACK,     /**< checkpoints and threshold read-back (safety_trips_check) */
    SAFETY_TB_PLAUS,        /**< sensor plausibility (frozen check) */
    SAFETY_TB_FEED,         /**< watchdog feed and interrupt-rate check */
    SAFETY_TB_COUNT
};

#define SAFETY_TIMING_MAGIC 0x54424C4BU   /* "KLBT" */

struct safety_timing {
    uint32_t magic;
    uint32_t periods;                    /**< control periods recorded since the last reset */
    uint32_t max[SAFETY_TB_COUNT];       /**< worst case per block, core cycles */
    uint64_t sum[SAFETY_TB_COUNT];       /**< sum per block, core cycles */
    uint32_t gate_lock_max;              /**< longest interrupt-locked window of the HRTIM output enable */
    volatile uint32_t reset;             /**< set to 1 to restart the record (cleared by the next period) */
};

#ifdef CONFIG_OWNTECH_SAFETY_TIMING_PROBES
extern volatile struct safety_timing g_safety_timing;
#endif

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_TIMING_H_ */
