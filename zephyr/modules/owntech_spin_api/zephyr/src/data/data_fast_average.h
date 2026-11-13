/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file data_fast_average.h
 * @brief Sum of the samples of one ADC rank written in a circular DMA buffer
 *        between two reads (SensorsAPI::getAverageValueFast, Core2 ADR 0007).
 *
 * No hardware access: the caller reads the DMA position and passes slots, so
 * the arithmetic is tested on the host (tests/l0/fast_average).
 */

#ifndef DATA_FAST_AVERAGE_H_
#define DATA_FAST_AVERAGE_H_

#include <stdint.h>

/**
 * @brief Sums the samples of one rank written after slot @p previous, up to
 *        and including slot @p newest.
 *
 * Slot i of the buffer holds the sample of rank (i mod n) + 1, so both slots
 * belong to the same rank and are a multiple of n apart. The samples summed are
 * newest, newest - n, ... down to (excluding) previous: from 0 (newest ==
 * previous, nothing new) to size / n - 1. Slots still holding @p empty (never
 * written) are skipped.
 *
 * @note A whole lap of the buffer between two reads is not detectable (it
 *       looks like fewer new samples); the caller reads more often than one lap.
 *
 * @param buffer          Circular DMA buffer.
 * @param size            Buffer size S, a multiple of n.
 * @param sequence_length Number n of channels in the ADC sequence.
 * @param newest          Newest slot of the rank now.
 * @param previous        Newest slot of the rank at the previous read.
 * @param empty           Value of a slot never written.
 * @param[out] raw_sum    Sum of the raw samples summed.
 * @return Number of samples summed (0: nothing new or only empty slots).
 */
static inline uint32_t data_fast_sum_new_samples(const volatile uint16_t* buffer,
												 uint32_t size,
												 uint32_t sequence_length,
												 uint32_t newest,
												 uint32_t previous,
												 uint16_t empty,
												 uint32_t* raw_sum)
{
	const uint32_t distance = (newest + size - previous) % size;
	uint32_t slot = newest;
	uint32_t count = 0;
	uint32_t sum = 0;
	for (uint32_t k = 0 ; k < distance / sequence_length ; k++)
	{
		const uint16_t raw = buffer[slot];
		if (raw != empty)
		{
			sum += raw;
			count++;
		}
		slot = (slot >= sequence_length) ? (slot - sequence_length)
										 : (slot + size - sequence_length);
	}
	*raw_sum = sum;
	return count;
}

#endif /* DATA_FAST_AVERAGE_H_ */
