/*
 * Copyright (c) 2021-present LAAS-CNRS
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
 * @date   2023
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 *
 * @brief Data dispatch is intended at dispatching ADCs
 * acquired data from DMA buffers to per-channel buffers.
 * User can then request the data of a specific channel.
 *
 * It uses double-buffering, holding 2 buffers for each
 * enabled channel of each ADC, one being filled and one
 * made available to the user.
 */

#ifndef DATA_DISPATCH_H_
#define DATA_DISPATCH_H_


/* Stdlib */
#include <stdint.h>


/* Constants */

const uint16_t PEEK_NO_VALUE = 0xFFFF;
const uint8_t CHANNELS_BUFFERS_SIZE = 32;
/* DMA buffers are filled with this value before the acquisition starts: a
 * 12-bit ADC cannot produce it, so a slot holding it was never written. */
const uint16_t DMA_EMPTY_SAMPLE = 0xFFFF;

/**
 * Dispatch method
 */
typedef enum {task, interrupt} dispatch_t;

/**
 * @brief Init function to be called first.
 *
 * @param dispatch_method Indicates when the dispatch
 *        should be done.
 * @param repetitions If dispatch is done at task start,
 *        this value represents the number of acquisitions
 *        that are done between two execution of the
 *        task. Ignored if dispatch is done on interrupt.
 */
void data_dispatch_init(dispatch_t dispatch_method, uint32_t repetitions);

/**
 * @brief Dispatch function: gets the readings and store them
 *        in per-channel arrays. This function is called by DMA
 *        callback when the DMA has filled one of its buffers.
 *
 * @param adc_number Number of the ADC from which data comes.
 */
void data_dispatch_do_dispatch(uint8_t adc_number);

/**
 * @brief Function to proceed to all chanels dispatch when
 *        it is done at uninterruptible task start.
 */
void data_dispatch_do_full_dispatch();

/**
 * @brief  Obtain data for a specific channel.
 *         The data is provided as an array of values
 *         and the count of data in this buffer is returned
 *         as an output parameter.
 *
 * @param  adc_number Number of the ADC from which to
 *         obtain data.
 * @param  channel_rank Rank of the channel from which
 *         to obtain data.
 * @param  number_of_values_acquired Output parameter:
 *         address to a variable that will be updated
 *         by the function with the data count.
 * @return Buffer containing the available data.
 *         Note that the returned buffer is invalidated
 *         by further calls to the function with same
 *         adc number/channel rank.
 */
uint16_t* data_dispatch_get_acquired_values(uint8_t adc_number,
                                            uint8_t channel_rank,
                                            uint32_t& number_of_values_acquired);

/**
 * @brief  Peek data for a specific channel:
 *         obtain the latest value from the channel without
 *         removing it from the buffer.
 *
 * @param  adc_number Number of the ADC from which to
 *         obtain data.
 * @param  channel_rank Rank of the channel from which
 *         to obtain data.
 * @return Latest available value from the buffer.
 *         Note that if no value has been acquired,
 *         returned value will be 0.
 */
uint16_t data_dispatch_peek_acquired_value(uint8_t adc_number,
                                           uint8_t channel_rank);


/**
 * @brief  Circular DMA buffer of an ADC, for the latest-sample reads
 *         (DataAPI fast channels). Only in task mode, where the DMA buffer
 *         size is a multiple of the sequence length, so that slot i always
 *         holds the sample of rank (i mod channel count) + 1.
 *
 * @param[in]  adc_number    ADC number (1-5)
 * @param[out] buffer        DMA buffer
 * @param[out] buffer_size   Buffer size in samples
 * @param[out] channel_count Sequence length of the ADC
 * @return 0 on success, -1 if the ADC has no DMA buffer or the dispatch is
 *         not in task mode.
 */
int8_t data_dispatch_get_dma_buffer(uint8_t adc_number,
                                    volatile const uint16_t*& buffer,
                                    uint32_t& buffer_size,
                                    uint8_t& channel_count);

/**
 * @brief  Restart an ADC's acquisition from the start of its DMA buffer,
 *         which restores the slot-to-rank mapping after an ADC overrun:
 *         stop the ADC, restart the DMA channel at slot 0, empty the buffer,
 *         clear the overrun flag, restart the ADC. Cold path, thread context.
 *
 * @param[in] adc_number ADC number (1-5)
 */
void data_dispatch_resync(uint8_t adc_number);

#endif /* DATA_DISPATCH_H_ */
