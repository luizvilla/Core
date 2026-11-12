/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file timing_probe.h
 * @brief Cycle-counter probes inside controlStep() for the on-target timing tests (L0-TIM, ADR 0005).
 *
 * TIMING_PROBE(i) stores the Cortex-M4 DWT cycle counter in timing::probe_cycles[i]. It expands to nothing
 * unless TIMING_HARNESS is defined (PlatformIO env TIMING only), so the production firmware, the host L0 tests
 * and the L1 C-Block are unchanged.
 */
#ifndef TIMING_PROBE_H
#define TIMING_PROBE_H

#include <stdint.h>

namespace timing {

/** @brief Probe points in controlStep(): entry, after the boost cold loop block, after the hot loop, before
 * calculateDuty, after it, after the grid observer, before return. */
inline constexpr uint8_t probe_count = 7U;

#ifdef TIMING_HARNESS
/** @brief DWT->CYCCNT at each probe point of the last controlStep() call. */
extern volatile uint32_t probe_cycles[probe_count];

/** @brief Probe points of one critical tick (main.cpp criticalTickBody): start, after the sensors, after
 * controlStep, after PWM and relay, end (after the scope). */
inline constexpr uint8_t tick_probe_count = 5U;
/** @brief DWT->CYCCNT at each tick probe point of the last tick. */
extern volatile uint32_t tick_cycles[tick_probe_count];

/** @brief Probe points in the PWM and relay writes (main.cpp criticalTickBody): start, after the boost duty,
 * after the boost enable, after burst, after the bridge duty, after the bridge enables, after the PWM acknowledgement
 * to the inverter, after the relay. */
inline constexpr uint8_t apply_probe_count = 8U;
/** @brief DWT->CYCCNT at each apply probe point of the last tick. */
extern volatile uint32_t apply_cycles[apply_probe_count];

/** @brief Stores the cycle counter (DWT->CYCCNT, 0xE0001004) for probe point @p i. */
inline void probe(uint8_t i)
{
    probe_cycles[i] = *reinterpret_cast<volatile const uint32_t *>(0xE0001004UL);
}
#endif

}  // namespace timing

#ifdef TIMING_HARNESS
#define TIMING_PROBE(i) ::timing::probe(i)
#define TIMING_TICK_PROBE(i) (::timing::tick_cycles[(i)] = *reinterpret_cast<volatile const uint32_t *>(0xE0001004UL))
#define TIMING_APPLY_PROBE(i) (::timing::apply_cycles[(i)] = *reinterpret_cast<volatile const uint32_t *>(0xE0001004UL))
#else
#define TIMING_PROBE(i) ((void)0)
#define TIMING_TICK_PROBE(i) ((void)0)
#define TIMING_APPLY_PROBE(i) ((void)0)
#endif

#endif  // TIMING_PROBE_H
