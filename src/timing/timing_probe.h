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

/** @brief Stores the cycle counter (DWT->CYCCNT, 0xE0001004) for probe point @p i. */
inline void probe(uint8_t i)
{
    probe_cycles[i] = *reinterpret_cast<volatile const uint32_t *>(0xE0001004UL);
}
#endif

}  // namespace timing

#ifdef TIMING_HARNESS
#define TIMING_PROBE(i) ::timing::probe(i)
#else
#define TIMING_PROBE(i) ((void)0)
#endif

#endif  // TIMING_PROBE_H
