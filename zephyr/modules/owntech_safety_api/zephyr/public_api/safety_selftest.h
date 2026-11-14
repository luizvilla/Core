/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_selftest.h
 * @brief Class B self-tests (IEC 60730-1 Annex H; ADR 0009, #21).
 */

#ifndef SAFETY_SELFTEST_H_
#define SAFETY_SELFTEST_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Which self-test failed (detail of SAFETY_FAULT_SELFTEST). */
enum {
    SAFETY_SELFTEST_CPU       = 1,
    SAFETY_SELFTEST_RAM       = 2,
    SAFETY_SELFTEST_FLASH     = 3,
    SAFETY_SELFTEST_CLOCK     = 4,
    SAFETY_SELFTEST_VREF      = 5,
    SAFETY_SELFTEST_IRQ_RATE  = 6,   /**< control task interrupts far more frequent than declared */
    SAFETY_SELFTEST_FROZEN    = 7,   /**< a current sensor reading frozen while the PWM runs (| index << 8) */
    SAFETY_SELFTEST_OFFSET    = 8,   /**< a current sensor away from zero at idle (| index << 8) */
};

#define SAFETY_SELFTEST_MAGIC 0x54535453U   /* "STST" */

/** Results, readable over SWD by the test harness. */
struct safety_selftest_status
{
    uint32_t magic;
    uint32_t boot_passed;       /**< b0 CPU, b1 RAM, b2 flash reference taken, b3 clock, b4 VREFINT */
    uint32_t boot_us[4];        /**< CPU, RAM, flash CRC, clock: duration at start-up */
    uint32_t boot_ram_bytes;
    uint32_t flash_bytes;
    uint32_t flash_reference;
    uint32_t clock_counts;      /**< TIM5 clocks over 8 LSI periods */
    uint32_t vrefint;
    uint32_t vdda_mv;           /**< VREF+ from VREFINT (the VREFBUF voltage when it is on, else VDDA), mV */
    uint32_t runtime_cpu;
    uint32_t runtime_ram_passes;
    uint32_t runtime_flash_passes;
    uint32_t runtime_clock;
    uint32_t runtime_max_us;    /**< longest run-time slice */
    uint32_t last_failure;
};

extern volatile struct safety_selftest_status g_safety_selftest;

/** Fault injection for the L0 tests (CONFIG_OWNTECH_SAFETY_TEST_HOOKS only). */
#define SAFETY_SELFTEST_INJECT_RAM    (1U << 0)
#define SAFETY_SELFTEST_INJECT_FLASH  (1U << 1)
#define SAFETY_SELFTEST_INJECT_CLOCK  (1U << 2)
#define SAFETY_SELFTEST_INJECT_CPU    (1U << 3)
#define SAFETY_SELFTEST_INJECT_FLOW   (1U << 4)

void safety_selftest_inject(uint32_t what);
uint32_t safety_selftest_injected(void);

/** @brief Plausibility of the current sensors, once per control period (safety_data.cpp). */
void safety_plausibility_period(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_SELFTEST_H_ */
