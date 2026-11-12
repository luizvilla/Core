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
 * @date   2024
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Thomas Walter <thomas.walter@laas.fr>
 */


/* Stdlib */
#include <string.h>

/* Current class header */
#include "DataAPI.h"

/* OwnTech Power API */
#include "SpinAPI.h"
#include "adc.h"

/* Current module private functions */
#include "./data/data_dispatch.h"
#include "./data/dma.h"
#include "owntech_fast_code.h"
#include "owntech_timing_probe.h"

#ifdef TIMING_HARNESS
volatile uint32_t timing_read_t0;
volatile uint32_t timing_read_cycles[3];
#endif

/**
 *  Static class members
 */

bool DataAPI::is_started = false;
bool DataAPI::adcInitialized = false;
uint8_t DataAPI::channels_ranks[ADC_COUNT][CHANNELS_PER_ADC] = {0};
uint8_t DataAPI::current_rank[ADC_COUNT] = {0};
DispatchMethod_t DataAPI::dispatch_method = DispatchMethod_t::on_dma_interrupt;
uint32_t DataAPI::repetition_count_between_dispatches = 0;
float32_t*** DataAPI::converted_values_buffer = nullptr;
volatile uint8_t DataAPI::fast_resync_pending[ADC_COUNT] = {0};
volatile uint32_t DataAPI::fast_overrun_count = 0;


adc_t DataAPI::current_adc[PIN_COUNT] = {DEFAULT_ADC};

/**
 *  Public functions accessible only when using a power shield
 */

/* Public functions */

int8_t DataAPI::enableAcquisition(uint8_t pin_num, adc_t adc_num)
{
	if (adc_num == DEFAULT_ADC)
	{
		adc_num = DataAPI::getDefaultAdcForPin(pin_num);
	}

	if (adc_num == UNKNOWN_ADC)
	{
		return -1;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return -1;
	}

	int8_t err = this->enableChannel(adc_num, channel_num);
	if (err == 0)
	{
		DataAPI::current_adc[pin_num-1] = adc_num;
	}

	return err;
}

int8_t DataAPI::start()
{
	if (DataAPI::is_started == true)
		return -1;

	/* Initialize conversion */
	data_conversion_init();

	/* Initialize data dispatch */
	switch (this->dispatch_method)
	{
		case DispatchMethod_t::on_dma_interrupt:
			/* Dispatch is handled automatically by Data Dispatch on interrupt */
			data_dispatch_init(interrupt, 0);
			break;
		case DispatchMethod_t::externally_triggered:
			/* Dispatch is triggered by an external call */
			if (this->repetition_count_between_dispatches == 0)
				return -1;

			data_dispatch_init(task, this->repetition_count_between_dispatches);
	}

	/* Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/* Launch ADC conversion */
	adc_start();

	DataAPI::is_started = true;

	return 0;
}

bool DataAPI::started()
{
	return DataAPI::is_started;
}

int8_t DataAPI::stop()
{
	if (DataAPI::is_started != true)
		return -1;

	adc_stop();

	/* Free buffers storage */
	if (DataAPI::converted_values_buffer != nullptr)
	{
		for (int adc_index = 0 ; adc_index < ADC_COUNT ; adc_index++)
		{
			if (DataAPI::converted_values_buffer[adc_index] != nullptr)
			{
				for (int channel_index = 0 ;
					 channel_index < CHANNELS_PER_ADC ;
					 channel_index++)
				{
					delete DataAPI::converted_values_buffer[adc_index][channel_index];
				}
				delete DataAPI::converted_values_buffer[adc_index];
			}
		}
		delete DataAPI::converted_values_buffer;
		DataAPI::converted_values_buffer = nullptr;
	}

	DataAPI::is_started = false;

	return 0;
}

void DataAPI::triggerAcquisition(adc_t adc_num)
{
	/*Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/*  Proceed */
	uint8_t enabled_channels =
				adc_get_enabled_channels_count(adc_num);
	adc_trigger_software_conversion(adc_num, enabled_channels);
}

uint16_t* DataAPI::getRawValues(uint8_t pin_num,
								uint32_t& number_of_values_acquired)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	return this->getChannelRawValues(adc_num,
									 channel_num,
									 number_of_values_acquired);
}

float32_t* DataAPI::getValues(uint8_t pin_number,
							  uint32_t& number_of_values_acquired)
{
	adc_t adc_number = DataAPI::getCurrentAdcForPin(pin_number);
	if (adc_number == UNKNOWN_ADC)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	uint8_t channel_num = this->getChannelNumber(adc_number, pin_number);
	if (channel_num == 0)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	return this->getChannelValues(adc_number,
								  channel_num,
								  number_of_values_acquired);
}

float32_t DataAPI::peekLatestValue(uint8_t pin_num)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return NO_VALUE;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return NO_VALUE;
	}

	return this->peekChannel(adc_num, channel_num);
}

float32_t DataAPI::getLatestValue(uint8_t pin_num, uint8_t* dataValid)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		if (dataValid != nullptr)
		{
			*dataValid = DATA_IS_MISSING;
		}
		return NO_VALUE;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		if (dataValid != nullptr)
		{
			*dataValid = DATA_IS_MISSING;
		}
		return NO_VALUE;
	}

	return this->getChannelLatest(adc_num, channel_num, dataValid);
}

float32_t DataAPI::convertValue(uint8_t pin_num, uint16_t raw_value)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	return data_conversion_convert_raw_value(adc_num, channel_num, raw_value);
}

void DataAPI::setConversionParametersLinear(uint8_t pin_num,
											float32_t gain,
											float32_t offset)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return;
	}

	data_conversion_set_conversion_parameters_linear(adc_num,
													 channel_num,
													 gain,
													 offset);
}

void DataAPI::setConversionParametersNtcThermistor(uint8_t pin_num,
												   float32_t r0,
												   float32_t b,
												   float32_t rdiv,
												   float32_t t0)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return;
	}

	data_conversion_set_conversion_parameters_therm(adc_num,
													channel_num,
													r0,
													b,
													rdiv,
													t0);
}


float32_t DataAPI::getConversionParameterValue(uint8_t pin_num,
											   parameter_t parameter_name)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	return data_conversion_get_parameter(adc_num,channel_num, parameter_name);
}

conversion_type_t DataAPI::getConversionParameterType(uint8_t pin_num)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return no_channel_error;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return no_channel_error;
	}

	return data_conversion_get_conversion_type(adc_num,channel_num);
}

int8_t DataAPI::storeConversionParametersInMemory(uint8_t pin_num)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	return data_conversion_store_channel_parameters_in_nvs(adc_num, channel_num);
}


int8_t DataAPI::retrieveConversionParametersFromMemory(uint8_t pin_num)
{
	adc_t adc_num = DataAPI::getCurrentAdcForPin(pin_num);
	if (adc_num == UNKNOWN_ADC)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	uint8_t channel_num = this->getChannelNumber(adc_num, pin_num);
	if (channel_num == 0)
	{
		return ERROR_CHANNEL_NOT_FOUND;
	}

	return data_conversion_retrieve_channel_parameters_from_nvs(adc_num,
																channel_num);
}

void DataAPI::configureDiscontinuousMode(adc_t adc_number,
										 uint32_t discontinuous_count)
{
	/* Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/* Proceed */
	adc_configure_discontinuous_mode(adc_number, discontinuous_count);
}

void DataAPI::configureTriggerSource(adc_t adc_number,
									 trigger_source_t trigger_source)
{
	/**
	 *  Check parameters validity
	 */

	if ( (adc_number == UNKNOWN_ADC) || (adc_number == DEFAULT_ADC) ) return;

	/* Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/* Proceed */

	if (trigger_source == TRIG_SOFTWARE)
	{
		adc_configure_trigger_source(adc_number, software);
	}
	else /* (trigger_source == TRIG_PWM) */
	{
		adc_ev_src_t event;
		switch(adc_number)
		{
			case ADC_1:
				event = hrtim_ev1;
				break;
			case ADC_2:
				event = hrtim_ev3;
				break;
			case ADC_3:
				event = hrtim_ev5;
				break;
			case ADC_4:
				event = hrtim_ev7;
				break;
			case ADC_5:
				event = hrtim_ev9;
				break;
			case UNKNOWN_ADC:
			case DEFAULT_ADC:
			default:
				return;
		}
		adc_configure_trigger_source(adc_number, event);
	}
}

/* Private functions */

void DataAPI::initializeAllAdcs()
{
	if (DataAPI::adcInitialized == false)
	{
		/* Perform default configuration */
		adc_configure_trigger_source(1, software);
		adc_configure_trigger_source(2, software);
		adc_configure_trigger_source(3, software);
		adc_configure_trigger_source(4, software);
		adc_configure_trigger_source(5, software);

		DataAPI::adcInitialized = true;
	}
}

int8_t DataAPI::enableChannel(adc_t adc_num, uint8_t channel_num)
{
	if (DataAPI::is_started == true)
		return -1;

	if ( (adc_num == 0) || (adc_num > ADC_COUNT) )
		return -1;

	if ( (channel_num == 0) || (channel_num > CHANNELS_PER_ADC) )
		return -1;


	/* Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/* Enable DMA */
	adc_configure_use_dma(adc_num, true);

	/* Set channel for activation */
	adc_add_channel(adc_num, channel_num);


	/* Remember rank */
	uint8_t adc_index = adc_num-1;
	uint8_t channel_index = channel_num-1;
	DataAPI::current_rank[adc_index]++;
	DataAPI::channels_ranks[adc_index][channel_index] =
											DataAPI::current_rank[adc_index];

	return 0;
}

void DataAPI::disableChannel(adc_t adc_num, uint8_t channel)
{
	/* Make sure module is initialized */
	if (adcInitialized == false)
	{
		initializeAllAdcs();
	}

	/* Proceed */
	adc_remove_channel(adc_num, channel);
}

uint16_t* DataAPI::getChannelRawValues(adc_t adc_num,
									   uint8_t channel_num,
									   uint32_t& number_of_values_acquired)
{
	if (DataAPI::is_started == false)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	uint8_t channel_rank = DataAPI::getChannelRank(adc_num, channel_num);
	if (channel_rank == 0)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	return data_dispatch_get_acquired_values(adc_num,
											 channel_rank,
											 number_of_values_acquired);
}

float32_t* DataAPI::getChannelValues(adc_t adc_number,
									 uint8_t channel_num,
									 uint32_t& number_of_values_acquired)
{
	/* Check that API is started */
	if (DataAPI::is_started == false)
	{
		number_of_values_acquired = 0;
		return nullptr;
	}

	/* Get raw values */
	uint16_t* raw_values =
				DataAPI::getChannelRawValues(adc_number,
											 channel_num,
											 number_of_values_acquired);

	if (number_of_values_acquired == 0)
	{
		return nullptr;
	}

	/* At least one value to convert: make sure a buffer is available */
	uint8_t adc_index = (uint8_t)adc_number - 1;
	uint8_t channel_index = channel_num - 1;
	if (DataAPI::converted_values_buffer == nullptr)
	{
		DataAPI::converted_values_buffer = new float32_t**[ADC_COUNT];
		for (int i = 0 ; i < ADC_COUNT ; i++)
		{
			DataAPI::converted_values_buffer[i] = nullptr;
		}
	}
	if (DataAPI::converted_values_buffer[adc_index] == nullptr)
	{
		DataAPI::converted_values_buffer[adc_index] =
										new float32_t*[CHANNELS_PER_ADC];

		for (int i = 0 ; i < CHANNELS_PER_ADC ; i++)
		{
			DataAPI::converted_values_buffer[adc_index][i] = nullptr;
		}
	}
	if (DataAPI::converted_values_buffer[adc_index][channel_index] == nullptr)
	{
		DataAPI::converted_values_buffer[adc_index][channel_index] =
										new float32_t[CHANNELS_BUFFERS_SIZE];
	}

	/* Proceed to conversion */
	for (uint32_t i = 0 ; i < number_of_values_acquired ; i++)
	{
		DataAPI::converted_values_buffer[adc_index][channel_index][i] =
					data_conversion_convert_raw_value(adc_number,
													  channel_num,
													  raw_values[i]);
	}

	/* Return converted values buffer */
	return DataAPI::converted_values_buffer[adc_index][channel_index];
}

OWNTECH_FAST_CODE float32_t DataAPI::peekChannel(adc_t adc_num, uint8_t channel_num)
{
	if (DataAPI::is_started == false)
	{
		return NO_VALUE;
	}

	uint8_t channel_rank = DataAPI::getChannelRank(adc_num, channel_num);
	if (channel_rank == 0)
	{
		return NO_VALUE;
	}

	uint16_t raw_value = data_dispatch_peek_acquired_value(adc_num,
														   channel_rank);
	if (raw_value == PEEK_NO_VALUE)
	{
		return NO_VALUE;
	}

	return data_conversion_convert_raw_value(adc_num, channel_num, raw_value);
}

OWNTECH_FAST_CODE float32_t DataAPI::getChannelLatest(adc_t adc_num,
									uint8_t channel_num,
									uint8_t* dataValid)
{
	if (DataAPI::is_started == false)
	{
		if (dataValid != nullptr)
		{
			*dataValid = DATA_IS_MISSING;
		}
		return NO_VALUE;
	}

	uint8_t channel_rank = DataAPI::getChannelRank(adc_num, channel_num);
	if (channel_rank == 0)
	{
		if (dataValid != nullptr)
		{
			*dataValid = DATA_IS_MISSING;
		}
		return NO_VALUE;
	}

	uint32_t data_count;
	OT_TIMING_MARK(t1);
	uint16_t* buffer = data_dispatch_get_acquired_values(adc_num,
														 channel_rank,
														 data_count);
	OT_TIMING_MARK(t2);

	if (data_count > 0)
	{
		uint16_t raw_value = buffer[data_count - 1];
		if (dataValid != nullptr)
		{
			*dataValid = DATA_IS_OK;
		}
		const float32_t value = data_conversion_convert_raw_value(adc_num,
																  channel_num,
																  raw_value);
		OT_TIMING_READ_END(t1, t2);
		return value;
	}
	else
	{
		uint16_t raw_value = data_dispatch_peek_acquired_value(adc_num,
															   channel_rank);
		OT_TIMING_MARK(t2_peek);

		float32_t peekValue;
		if (raw_value != PEEK_NO_VALUE)
		{
			peekValue = data_conversion_convert_raw_value(adc_num,
														  channel_num,
														  raw_value);
		}
		else
		{
			peekValue = NO_VALUE;
		}

		if (dataValid != nullptr)
		{
			if (peekValue != NO_VALUE)
			{
				*dataValid = DATA_IS_OLD;
			}
			else
			{
				*dataValid = DATA_IS_MISSING;
			}
		}
		OT_TIMING_READ_END(t1, t2_peek);
		return peekValue;
	}
}

static_assert(DATA_FAST_EMPTY_SAMPLE == DMA_EMPTY_SAMPLE,
			  "fast reads and dispatch must agree on the empty DMA sample");

int8_t DataAPI::prepareFastChannel(adc_t adc_num,
								   uint8_t channel_num,
								   DataFastChannel& channel)
{
	static volatile uint32_t* const adc_isr_registers[ADC_COUNT] =
	{
		&(ADC1->ISR), &(ADC2->ISR), &(ADC3->ISR), &(ADC4->ISR), &(ADC5->ISR)
	};

	if (DataAPI::is_started == false)
		return -1;

	uint8_t channel_rank = DataAPI::getChannelRank(adc_num, channel_num);
	if (channel_rank == 0)
		return -1;

	if (data_conversion_get_conversion_type(adc_num, channel_num) !=
		conversion_linear)
		return -1;

	volatile const uint16_t* buffer;
	uint32_t size;
	uint8_t count;
	if (data_dispatch_get_dma_buffer(adc_num, buffer, size, count) != 0)
		return -1;
	if ( (size > DATA_FAST_MAX_DMA_SIZE) || (size % count != 0) ||
		 (channel_rank > count) )
		return -1;

	uint8_t adc_index = adc_num - 1;
	uint32_t rank_index = channel_rank - 1;

	/**
	 * CNDTR = c: next write at slot p = (S - c) mod S, newest written slot
	 * p - 1 (mod S). The newest slot of the rank is the last slot at or
	 * before it with (slot mod n) == rank_index.
	 */
	for (uint32_t c = 0 ; c <= size ; c++)
	{
		uint32_t newest = (2 * size - c - 1) % size;
		uint32_t back = (newest + count - rank_index) % count;
		channel.newest_slot[c] = (uint8_t)((newest + size - back) % size);
	}

	channel.dma_buffer     = buffer;
	channel.dma_remaining  = dma_get_remaining_register(adc_num);
	channel.adc_isr        = adc_isr_registers[adc_index];
	channel.resync_pending = &DataAPI::fast_resync_pending[adc_index];
	channel.gain           = data_conversion_get_parameter(adc_num,
														   channel_num, 1);
	channel.offset         = data_conversion_get_parameter(adc_num,
														   channel_num, 2);
	channel.last_remaining = UINT32_MAX;
	channel.adc_index      = adc_index;
	channel.dma_size       = (uint8_t)size;

	return 0;
}

OWNTECH_FAST_CODE void DataAPI::noteFastOverrun(uint8_t adc_index)
{
	if (DataAPI::fast_resync_pending[adc_index] == 0)
	{
		DataAPI::fast_resync_pending[adc_index] = 1;
		DataAPI::fast_overrun_count = DataAPI::fast_overrun_count + 1;
	}
}

void DataAPI::resyncFastChannels()
{
	for (uint8_t adc_index = 0 ; adc_index < ADC_COUNT ; adc_index++)
	{
		if (DataAPI::fast_resync_pending[adc_index] != 0)
		{
			/**
			 * The reads of this ADC return DATA_IS_MISSING while pending. An
			 * overrun that occurs again after the restart keeps its flag set,
			 * so the next read reports it again.
			 */
			uint8_t adc_num = adc_index + 1;
			data_dispatch_resync(adc_num);
			DataAPI::fast_resync_pending[adc_index] = 0;
		}
	}
}

uint32_t DataAPI::getFastOverrunCount()
{
	return DataAPI::fast_overrun_count;
}

uint8_t DataAPI::getChannelRank(adc_t adc_num, uint8_t channel_num)
{
	if ( (adc_num > ADC_COUNT) || (channel_num > CHANNELS_PER_ADC) )
		return 0;

	uint8_t adc_index = adc_num-1;
	uint8_t channel_index = channel_num-1;

	return DataAPI::channels_ranks[adc_index][channel_index];
}

uint8_t DataAPI::getChannelNumber(adc_t adc_num, uint8_t shield_pin)
{
	switch (adc_num)
	{
	case 1:
		switch (shield_pin)
		{
			case 1:
				return 14;
				break;
			case 2:
				return 11;
				break;
			case 5:
				return 5;
				break;
			case 24:
				return 6;
				break;
			case 25:
				return 7;
				break;
			case 26:
				return 8;
				break;
			case 27:
				return 9;
				break;
			case 29:
				return 1;
				break;
			case 30:
				return 2;
				break;
			case 31:
				return 5;
				break;
			case 37:
				return 12;
				break;
			case 50:
				return 3;
				break;
			case 51:
				return 4;
				break;
			default:
				return 0;
				break;
		}
		break;
	case 2:
		switch (shield_pin)
		{
			case 1:
				return 14;
				break;
			case 6:
				return 15;
				break;
			case 24:
				return 6;
				break;
			case 25:
				return 7;
				break;
			case 26:
				return 8;
				break;
			case 27:
				return 9;
				break;
			case 29:
				return 1;
				break;
			case 30:
				return 2;
				break;
			case 32:
				return 13;
				break;
			case 34:
				return 3;
				break;
			case 35:
				return 5;
				break;
			case 42:
				return 12;
				break;
			case 43:
				return 11;
				break;
			case 44:
				return 4;
				break;
			case 45:
				return 17;
				break;
			default:
				return 0;
				break;
		}
		break;
	case 3:
		switch (shield_pin)
		{
			case 4:
				return 5;
				break;
			case 31:
				return 12;
				break;
			case 37:
				return 1;
				break;
			default:
				return 0;
				break;
		}
		break;
	case 4:
		switch (shield_pin)
		{
			case 2:
				return 3;
				break;
			case 5:
				return 4;
				break;
			case 6:
				return 5;
				break;
			default:
				return 0;
				break;
		}
		break;
	case 5:
		switch (shield_pin)
		{
			case 12:
				return 1;
				break;
			case 14:
				return 2;
				break;
			default:
				return 0;
				break;
		}
		break;
	default:
		return 0;
		break;
	}
}

adc_t DataAPI::getDefaultAdcForPin(uint8_t pin_number)
{
	switch (pin_number)
	{
		/* These pins allow only ADC 1 */
		case 51:
		case 52:
		/* These pins allow ADC 1 and ADC 2: default to ADC 1 */
		case 1:
		case 24:
		case 25:
		case 26:
		case 27:
		case 29:
		case 30:
		/* These pins allow ADC 1 and ADC 3: default to ADC 1 */
		case 31:
		case 37:
			return ADC_1;
			break;
		/* These pins allow only ADC 2 */
		case 32:
		case 34:
		case 35:
		case 42:
		case 43:
		case 44:
		case 45:
		/* This pin allows ADC 2 and ADC 4: default to ADC 2 */
		case 5:
			return ADC_2;
			break;
		/* This pin allows only ADC 3 */
		case 4:
			return ADC_3;
			break;
		/* This pin allows only ADC 4 */
		case 2:
			return ADC_4;
			break;
		/* This pin allows only ADC 5 */
		case 12:
		case 14:
			return ADC_5;
			break;
		default:
			return UNKNOWN_ADC;
	}
}

adc_t DataAPI::getCurrentAdcForPin(uint8_t pin_number)
{
	if ( (pin_number > 1) && (pin_number <=PIN_COUNT) )
	{
		adc_t currentAdc = DataAPI::current_adc[pin_number-1];

		if (currentAdc != DEFAULT_ADC)
		{
			return currentAdc;
		}
	}

	return UNKNOWN_ADC;
}

void DataAPI::setRepetitionsBetweenDispatches(uint32_t repetition)
{
	DataAPI::repetition_count_between_dispatches = repetition;
}

void DataAPI::setDispatchMethod(DispatchMethod_t dispatch_method)
{
	DataAPI::dispatch_method = dispatch_method;
}

OWNTECH_FAST_CODE void DataAPI::doFullDispatch()
{
	data_dispatch_do_full_dispatch();
}
