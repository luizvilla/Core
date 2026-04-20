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
 * @date   2023
 * @author Clément Foucher <clement.foucher@laas.fr>
 */



/* Current file header */
#include "TimerHAL.h"


static const struct device* timer4 = DEVICE_DT_GET(TIMER4_DEVICE);
static const struct device* timer3 = DEVICE_DT_GET(TIMER3_DEVICE);

bool TimerHAL::timer4init    = false;
bool TimerHAL::timer4started = false;
bool TimerHAL::timer3init    = false;
bool TimerHAL::timer3started = false;

static incremental_encoder_timer_config_t
timer_hal_get_default_incremental_encoder_config(timernumber_t timer_number)
{
	incremental_encoder_timer_config_t encoder_config =
	{
		.pin_mode = pull_up,
		.index_enable = encoder_index_enabled,
		.index_polarity = encoder_index_polarity_noninverted,
		.index_configuration = encoder_index_configuration_a_low_b_low
	};

	if (timer_number == TIMER3)
	{
		encoder_config.index_polarity = encoder_index_polarity_inverted;
		encoder_config.index_configuration =
			encoder_index_configuration_a_high_b_high;
	}

	return encoder_config;
}


void TimerHAL::Initialize(
	timernumber_t timer_number,
	const incremental_encoder_timer_config_t* encoder_config)
{
	incremental_encoder_timer_config_t effective_encoder_config =
		timer_hal_get_default_incremental_encoder_config(timer_number);

	if (encoder_config != nullptr)
	{
		effective_encoder_config = *encoder_config;
	}

	if (timer_number == TIMER4){
		if (device_is_ready(timer4) == true)
		{
			/* Configure timer */
			struct timer_config_t timer_cfg =
			{
				.timer_enable_irq = 0,
				.timer_enable_encoder = 1,
				.timer_enc_pin_mode = effective_encoder_config.pin_mode,
				.timer_encoder_index_enable = effective_encoder_config.index_enable,
				.timer_encoder_index_polarity = effective_encoder_config.index_polarity,
				.timer_encoder_index_configuration =
					effective_encoder_config.index_configuration

			};
			timer_config(timer4, &timer_cfg);
			timer4init = true;
		}
	}else{
		if (device_is_ready(timer3) == true)
		{
			/* Configure timer */
			struct timer_config_t timer_cfg =
			{
				.timer_enable_irq = 0,
				.timer_enable_encoder = 1,
				.timer_enc_pin_mode = effective_encoder_config.pin_mode,
				.timer_encoder_index_enable = effective_encoder_config.index_enable,
				.timer_encoder_index_polarity = effective_encoder_config.index_polarity,
				.timer_encoder_index_configuration =
					effective_encoder_config.index_configuration

			};
			timer_config(timer3, &timer_cfg);
			timer3init = true;
		}

	}
}

void TimerHAL::startLogIncrementalEncoder(
	timernumber_t timer_number,
	const incremental_encoder_timer_config_t* encoder_config)
{
	if(timer_number == TIMER4){
		if (timer4init == false)
		{
			Initialize(TIMER4, encoder_config);
		}

		if (timer4started == false)
		{
			if (device_is_ready(timer4) == true)
			{
				timer_start(timer4);
				timer4started = true;
			}
		}
	}else{
		if (timer3init == false)
		{
			Initialize(TIMER3, encoder_config);
		}

		if (timer3started == false)
		{
			if (device_is_ready(timer3) == true)
			{
				timer_start(timer3);
				timer3started = true;
			}
		}

	}
}

uint32_t TimerHAL::getIncrementalEncoderValue(timernumber_t timer_number)
{
	if(timer_number == TIMER4){
		if (timer4started == true)
		{
			return timer_get_count(timer4);
		}
		else
		{
			return 0;
		}
	}else{
		if (timer3started == true)
		{
			return timer_get_count(timer3);
		}
		else
		{
			return 0;
		}

	}
}
