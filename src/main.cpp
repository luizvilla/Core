/*
 * Copyright (c) 2024-present LAAS-CNRS
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

/**
 * @brief  This file implements cascaded FOC for an OwnTech OwnVerter board:
 *         an inner d/q current loop and an outer speed loop based on an
 *         AB+index incremental encoder selected through Shield Position API.
 *         Please check example documentation to get more details
 *         how to use this example: https://docs.owntech.org/examples/
 *
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 * @author Jean Alinei <jean.alinei@laas.fr>
 */

#include "app.h"

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "transform.h"
#include "zephyr/console/console.h"

void setup_routine();
void loop_background_task();
void loop_critical_task();
void application_task();

static void configure_scope();

static AppContext app;

static constexpr uint16_t SCOPE_SIZE = 512U;
static ScopeMimicry scope(SCOPE_SIZE, 19);

static enum control_state_mode control_state;
static float32_t control_state_f;
static uint8_t asked_mode = IDLEMODE;

bool mytrigger()
{
	return (control_state == POWER_ST);
}

static void configure_scope()
{
	scope.connectChannel(app.variable.V12_value, "V12_value");
	scope.connectChannel(app.variable.Vq, "Vq");
	scope.connectChannel(app.variable.Vd, "Vd");
	scope.connectChannel(app.variable.I1_low_value, "I1_low_value");
	scope.connectChannel(app.variable.I2_low_value, "I2_low_value");
	scope.connectChannel(app.variable.I_high, "I_high_value");
	scope.connectChannel(app.variable.Iq_meas, "Iq_meas");
	scope.connectChannel(app.variable.speed_meas_print, "speed_meas");
	scope.connectChannel(app.variable.speed_ref_print, "speed_ref");
	scope.connectChannel(app.variable.encoder_elec_angle, "encoder_angle");
	scope.connectChannel(app.variable.angle_filtered, "angle_filtered");
	scope.connectChannel(control_state_f, "control_state");
	scope.connectChannel(app.variable.electrical_offset_print, "electrical_offset");
	scope.connectChannel(app.variable.open_loop_mode_print, "open_loop_mode");
	scope.connectChannel(app.variable.theta_ol_print, "theta_ol");
	scope.connectChannel(app.variable.omega_ol_print, "omega_ol");
	scope.connectChannel(app.variable.vq_ol_print, "vq_ol");
	scope.connectChannel(app.variable.Idq_ref.d, "Id_ref");
	scope.connectChannel(app.variable.Idq_ref.q, "Iq_ref");
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();
}

void setup_routine()
{
	initialize_variable_defaults(app);

	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();
	app.variable.position_sensor_initialized = shield.position.initDefault();
	if (!app.variable.position_sensor_initialized) {
		printk("ERROR: failed to initialize the default position sensor from app.overlay.\n");
	}
	app.variable.active_position_type = shield.position.getActiveSensorType();

	configure_scope();

	init_motor_control(app);
	init_filters_and_regulators(app);

	asked_mode = IDLEMODE;
	control_state = IDLE_ST;
	restart_offset_calibration(app, asked_mode, IDLEMODE, control_state);
	spin.led.turnOn();

	uint32_t background_task_number =
		task.createBackground(loop_background_task);
	uint32_t app_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, app.setup.control_task_period);

	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();
}

void loop_background_task()
{
	app.variable.received_serial_char = console_getchar();
	switch (app.variable.received_serial_char) {
	case 'p':
		printk("power asked");
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'i':
		printk("idle asked");
		asked_mode = IDLEMODE;
		app.variable.speed_ref = 0.0F;
		break;
	case 'o':
		printk("offset recalibration asked");
		restart_offset_calibration(app, asked_mode, IDLEMODE, control_state);
		spin.led.turnOn();
		break;
	case 'r':
		app.variable.is_downloading = true;
		break;
	case 'u':
		app.variable.speed_ref += app.setup.speed_ref_step;
		if (app.variable.speed_ref > app.setup.speed_ref_max) {
			app.variable.speed_ref = app.setup.speed_ref_max;
		}
		break;
	case 'd':
		app.variable.speed_ref -= app.setup.speed_ref_step;
		if (app.variable.speed_ref < -app.setup.speed_ref_max) {
			app.variable.speed_ref = -app.setup.speed_ref_max;
		}
		break;
	case '[':
		adjust_electrical_offset(app, -app.setup.electrical_offset_step);
		scope.start();
		break;
	case ']':
		adjust_electrical_offset(app, app.setup.electrical_offset_step);
		scope.start();
		break;
	case 'z':
		app.variable.Idq_ref.q -= 0.1F;
		if (app.variable.Idq_ref.q < 0.0F) {
			app.variable.Idq_ref.q = 0.0F;
		}
		break;
	case 'x':
		app.variable.Idq_ref.q += 0.1F;
		if (app.variable.Idq_ref.q > app.setup.iq_ref_max) {
			app.variable.Idq_ref.q = app.setup.iq_ref_max;
		}
		break;
	case 'l':
	case 'L':
		toggle_open_loop_mode(app);
		break;
	case 'j':
	case 'J':
		app.variable.omega_ol -= app.setup.open_loop_speed_step;
		printk("open-loop speed = %.2f rad/s\n", (double)app.variable.omega_ol);
		break;
	case 'k':
	case 'K':
		app.variable.omega_ol += app.setup.open_loop_speed_step;
		printk("open-loop speed = %.2f rad/s\n", (double)app.variable.omega_ol);
		break;
	case 'n':
	case 'N':
		app.variable.vq_ol -= app.setup.open_loop_vq_step;
		printk("open-loop vq = %.2f V\n", (double)app.variable.vq_ol);
		break;
	case 'b':
	case 'B':
		app.variable.vq_ol += app.setup.open_loop_vq_step;
		printk("open-loop vq = %.2f V\n", (double)app.variable.vq_ol);
		break;
	case 't':
	case 'T':
		adjust_speed_loop_kp(app, -app.setup.speed_kp_step);
		break;
	case 'y':
	case 'Y':
		adjust_speed_loop_kp(app, app.setup.speed_kp_step);
		break;
	case 'g':
	case 'G':
		adjust_speed_loop_ki(app, -app.setup.speed_ki_step);
		break;
	case 'h':
	case 'H':
		adjust_speed_loop_ki(app, app.setup.speed_ki_step);
		break;
	case 'm':
		app.variable.memory_print = !app.variable.memory_print;
		break;
	case 'c':
	case 'C':
		shield.position.setAbzSpeedDecimation(app.variable.speed_decimation + 1U);
		break;
	case 'v':
	case 'V':
		shield.position.setAbzSpeedDecimation(app.variable.speed_decimation - 1U);
		break;
	case 's':
	case 'S':
		shield.position.setDirectionSign(-shield.position.getDirectionSign());
		break;
	case 'q':
		scope.start();
		break;
	}
}

void application_task()
{
	if (!app.variable.memory_print) {
		printk("%7.2f:", (double)app.variable.V_high);
		printk("%7.2f:", (double)app.variable.Iq_max);
		printk("%7.2f:", (double)app.variable.speed_ref);
		printk("%7.2f:", (double)app.variable.w_meas);
		printk("%7.2f:", (double)app.variable.I1_offset);
		printk("%7d:", control_state);
		printk("%7u:", app.variable.encoder_count);
		printk("%7ld:", (long)app.variable.encoder_delta_count);
		printk("%7.2f:", (double)shield.position.getElectricalOffset());
		printk("%7d:", shield.position.getDirectionSign());
		printk("%7d:", app.variable.open_loop_mode ? 1 : 0);
		printk("%7.2f:", (double)app.variable.angle_filtered);
		printk("%7.2f:", (double)app.variable.theta_ol);
		printk("%7.2f:", (double)app.variable.omega_ol);
		printk("%7.2f:", (double)app.variable.Idq_ref.q);
		printk("%7.2f\n", (double)app.variable.vq_ol);
	} else {
		app.variable.k_app_idx = (app.variable.k_app_idx + 1U) % SCOPE_SIZE;
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 0));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 1));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 2));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 3));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 4));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 5));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 6));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 7));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 8));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 9));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 10));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 11));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 12));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 13));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 14));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 15));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 16));
		printk("%.2f:", (double)scope.get_channel_value(app.variable.k_app_idx, 17));
		printk("%.2f", (double)scope.get_channel_value(app.variable.k_app_idx, 18));
		printk("\n");
	}

	if (app.variable.is_downloading) {
		dump_scope_datas(scope);
		app.variable.is_downloading = false;
	}

	switch (control_state) {
	case OFFSET_ST:
		if (app.variable.counter_time > (uint32_t)app.setup.nb_offset) {
			spin.led.turnOff();
			app.variable.I1_offset = -app.variable.tmpI1_offset / app.setup.nb_offset;
			app.variable.I2_offset = -app.variable.tmpI2_offset / app.setup.nb_offset;
			app.variable.position_data_valid = false;
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) &&
			app.variable.position_sensor_initialized &&
			app.variable.position_data_valid &&
			(app.variable.V_high_filtered > app.setup.v_high_min)) {
			control_state = POWER_ST;
		}
		spin.led.turnOn();
		break;

	case POWER_ST:
		if (asked_mode == IDLEMODE) {
			control_state = IDLE_ST;
		}
		spin.led.turnOff();
		break;

	case ERROR_ST:
		if (asked_mode == IDLEMODE) {
			app.variable.error_counter = 0U;
			control_state = IDLE_ST;
		}
		break;
	}

	task.suspendBackgroundMs(250);
}

void loop_critical_task()
{
	app.variable.counter_time++;

	retrieve_analog_data(app, control_state == OFFSET_ST);
	update_position_and_speed(app);

	if ((control_state == POWER_ST) && !app.variable.position_data_valid) {
		control_state = ERROR_ST;
	}

	if (update_overcurrent_error(app)) {
		control_state = ERROR_ST;
	}

	switch (control_state) {
	case OFFSET_ST:
	case IDLE_ST:
	case ERROR_ST:
		stop_pwm_and_reset_states_if_needed(app);
		break;
	case POWER_ST:
		if (app.variable.open_loop_mode) {
			control_open_loop(app);
		} else {
			control_speed(app);
		}
		compute_duties(app);
		apply_duties(app);
		start_pwms_if_needed(app);
		break;
	}

	if (app.variable.counter_time % app.setup.decimation == 0U) {
		update_scope_mirrors(app, control_state, control_state_f);
		scope.acquire();
	}
}

int main(void)
{
	setup_routine();
	return 0;
}
