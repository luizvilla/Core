/*
 * Copyright (c) 2022-2023 LAAS-CNRS
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
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @date   2023
 * @author Clément Foucher <clement.foucher@laas.fr>
 */



// Current file header
#include "TimerHAL.h"


static const struct device* timer4 = DEVICE_DT_GET(TIMER4_DEVICE);

bool TimerHAL::timer4init    = false;
bool TimerHAL::timer4started = false;


void TimerHAL::timer4Initialize()
{
	if (device_is_ready(timer4) == true)
	{
		// Configure timer
		struct timer_config_t timer_cfg =
		{
			.timer_enable_irq = 0,
			.timer_enable_encoder = 1,
			.timer_enc_pin_mode = pull_down,

		};
		timer_config(timer4, &timer_cfg);
		timer4init = true;
	}
}

void TimerHAL::startLogTimer4IncrementalEncoder()
{
	if (timer4init == false)
	{
		timer4Initialize();
	}

	if (timer4started == false)
	{
		if (device_is_ready(timer4) == true)
		{
			timer_start(timer4);
			timer4started = true;
		}
	}
}

uint32_t TimerHAL::getTimer4IncrementalEncoderValue()
{
	if (timer4started == true)
	{
		return timer_get_count(timer4);
	}
	else
	{
		return 0;
	}
}

static const struct device* timer3 = DEVICE_DT_GET(TIMER3_DEVICE);

bool TimerHAL::timer3init    = false;
bool TimerHAL::timer3started = false;


void TimerHAL::timer3Initialize()
{
	if (device_is_ready(timer3) == true)
	{
		// Configure timer
		struct timer_config_t timer_cfg =
		{
			.timer_enable_irq = 0,
			.timer_enable_encoder = 1,
			.timer_enc_pin_mode = pull_down,

		};
		timer_config(timer3, &timer_cfg);
		timer3init = true;
	}
}

void TimerHAL::startLogTimer3IncrementalEncoder()
{
	if (timer3init == false)
	{
		timer3Initialize();
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

uint32_t TimerHAL::getTimer3IncrementalEncoderValue()
{
	if (timer3started == true)
	{
		return timer_get_count(timer3);
	}
	else
	{
		return 0;
	}
}