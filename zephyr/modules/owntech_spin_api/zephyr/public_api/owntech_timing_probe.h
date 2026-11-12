/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/*
 * Cycle-counter probes in the ADC data path, for the Core2 on-target timing tests (L0-TIM-03, ADR 0005).
 * They exist only in builds with TIMING_HARNESS (env TIMING); everywhere else every macro is empty.
 *
 * - Data dispatch (data_dispatch.cpp): cycles of data_dispatch_do_dispatch() per ADC and the samples it
 *   copied, for the last dispatch.
 * - Latest-value reads (Sensors.cpp, DataAPI.cpp): each SensorsAPI::getLatestValue() adds its cost, split
 *   in three layers, to timing_read_cycles[]; the harness clears the sums before each tick:
 *     [0] sensor lookup and DataAPI checks (SensorsAPI, getChannelLatest, channel rank)
 *     [1] buffer layer (data_dispatch_get_acquired_values, or the peek when no new sample)
 *     [2] conversion (data_conversion_convert_raw_value)
 */

#ifndef OWNTECH_TIMING_PROBE_H_
#define OWNTECH_TIMING_PROBE_H_

#include <stdint.h>

#ifdef TIMING_HARNESS

#define OT_TIMING_ADC_COUNT 5U

extern volatile uint32_t timing_dispatch_cycles[OT_TIMING_ADC_COUNT];
extern volatile uint32_t timing_dispatch_samples[OT_TIMING_ADC_COUNT];
extern volatile uint32_t timing_read_t0;
extern volatile uint32_t timing_read_cycles[3];

#define OT_TIMING_CYCLES() (*(volatile const uint32_t *)0xE0001004UL)
#define OT_TIMING_MARK(var) const uint32_t var = OT_TIMING_CYCLES()
#define OT_TIMING_READ_START() (timing_read_t0 = OT_TIMING_CYCLES())
#define OT_TIMING_READ_END(t1, t2)                                   \
	do {                                                             \
		const uint32_t ot_t3 = OT_TIMING_CYCLES();                   \
		timing_read_cycles[0] = timing_read_cycles[0] + ((t1) - timing_read_t0); \
		timing_read_cycles[1] = timing_read_cycles[1] + ((t2) - (t1)); \
		timing_read_cycles[2] = timing_read_cycles[2] + (ot_t3 - (t2)); \
	} while (0)
#define OT_TIMING_DISPATCH_SAMPLES(adc_index, n) (timing_dispatch_samples[(adc_index)] = (uint32_t)(n))
#define OT_TIMING_DISPATCH_CYCLES(adc_index, t0) \
	(timing_dispatch_cycles[(adc_index)] = OT_TIMING_CYCLES() - (t0))

#else

#define OT_TIMING_MARK(var)
#define OT_TIMING_READ_START() ((void)0)
#define OT_TIMING_READ_END(t1, t2) ((void)0)
#define OT_TIMING_DISPATCH_SAMPLES(adc_index, n) ((void)0)
#define OT_TIMING_DISPATCH_CYCLES(adc_index, t0) ((void)0)

#endif

#endif /* OWNTECH_TIMING_PROBE_H_ */
