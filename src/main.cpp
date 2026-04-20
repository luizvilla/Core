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

static void restart_offset_calibration();
static void configure_scope();
static void update_scope_mirrors();

static AppContext app;

static constexpr uint16_t SCOPE_SIZE = 512U;
static ScopeMimicry scope(SCOPE_SIZE, 19);

enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

enum control_state_mode {
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

static enum control_state_mode control_state;
static float32_t control_state_f;
static uint8_t asked_mode = IDLEMODE;

bool mytrigger()
{
	return (control_state == POWER_ST);
}

static void restart_offset_calibration()
{
	stop_pwm_and_reset_states_if_needed(app);
	app.runtime.counter_time = 0U;
	app.runtime.encoder_count_prev = 0U;
	app.runtime.I1_offset = 0.0F;
	app.runtime.I2_offset = 0.0F;
	app.runtime.tmpI1_offset = 0.0F;
	app.runtime.tmpI2_offset = 0.0F;
	asked_mode = IDLEMODE;
	control_state = OFFSET_ST;
	spin.led.turnOn();
}

static void configure_scope()
{
	scope.connectChannel(app.runtime.V12_value, "V12_value");
	scope.connectChannel(app.runtime.Vq, "Vq");
	scope.connectChannel(app.runtime.Vd, "Vd");
	scope.connectChannel(app.runtime.I1_low_value, "I1_low_value");
	scope.connectChannel(app.runtime.I2_low_value, "I2_low_value");
	scope.connectChannel(app.runtime.I_high, "I_high_value");
	scope.connectChannel(app.runtime.Iq_meas, "Iq_meas");
	scope.connectChannel(app.runtime.speed_meas_print, "speed_meas");
	scope.connectChannel(app.runtime.speed_ref_print, "speed_ref");
	scope.connectChannel(app.runtime.encoder_elec_angle, "encoder_angle");
	scope.connectChannel(app.runtime.angle_filtered, "angle_filtered");
	scope.connectChannel(control_state_f, "control_state");
	scope.connectChannel(app.runtime.electrical_offset_print, "electrical_offset");
	scope.connectChannel(app.runtime.open_loop_mode_print, "open_loop_mode");
	scope.connectChannel(app.runtime.theta_ol_print, "theta_ol");
	scope.connectChannel(app.runtime.omega_ol_print, "omega_ol");
	scope.connectChannel(app.runtime.vq_ol_print, "vq_ol");
	scope.connectChannel(app.runtime.Idq_ref.d, "Id_ref");
	scope.connectChannel(app.runtime.Idq_ref.q, "Iq_ref");
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();
}

static void update_scope_mirrors()
{
	app.runtime.encoder_count_f = (float32_t)app.runtime.encoder_count;
	app.runtime.encoder_delta_count_f = (float32_t)app.runtime.encoder_delta_count;
	app.runtime.Va = app.runtime.Vabc.a;
	app.runtime.duty_a = app.runtime.duty_abc.a;
	app.runtime.duty_b = app.runtime.duty_abc.b;
	app.runtime.Iq_ref = app.runtime.Idq_ref.q;
	app.runtime.Id_ref = app.runtime.Idq_ref.d;
	app.runtime.Iq_meas = app.runtime.Idq.q;
	app.runtime.Vd = app.runtime.Vdq.d;
	app.runtime.Vq = app.runtime.Vdq.q;
	app.runtime.speed_ref_print = app.runtime.speed_ref;
	app.runtime.speed_meas_print = app.runtime.w_meas;
	app.runtime.electrical_offset_print = shield.position.getElectricalOffset();
	app.runtime.open_loop_mode_print = app.runtime.open_loop_mode ? 1.0F : 0.0F;
	app.runtime.theta_ol_print = app.runtime.theta_ol;
	app.runtime.omega_ol_print = app.runtime.omega_ol;
	app.runtime.vq_ol_print = app.runtime.vq_ol;
	app.runtime.Iabc_ref =
		Transform::to_threephase(app.runtime.Idq_ref, app.runtime.angle_4_control);
	app.runtime.Ia_ref = app.runtime.Iabc_ref.a;
	app.runtime.Ib_ref = app.runtime.Iabc_ref.b;
	app.runtime.counter_time_f = (float32_t)app.runtime.counter_time;
	control_state_f = (float32_t)control_state;
}

void setup_routine()
{
	initialize_runtime_defaults(app);

	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();
	app.runtime.position_sensor_initialized = shield.position.initDefault();
	if (!app.runtime.position_sensor_initialized) {
		printk("ERROR: failed to initialize the default position sensor from app.overlay.\n");
	}
	app.runtime.active_position_type = shield.position.getActiveSensorType();

	configure_scope();

	init_motor_control(app);
	init_filters_and_regulators(app);

	asked_mode = IDLEMODE;
	control_state = IDLE_ST;
	restart_offset_calibration();
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
	app.runtime.received_serial_char = console_getchar();
	switch (app.runtime.received_serial_char) {
	case 'p':
		printk("power asked");
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'i':
		printk("idle asked");
		asked_mode = IDLEMODE;
		app.runtime.speed_ref = 0.0F;
		break;
	case 'o':
		printk("offset recalibration asked");
		restart_offset_calibration();
		break;
	case 'r':
		app.runtime.is_downloading = true;
		break;
	case 'u':
		app.runtime.speed_ref += app.setup.speed_ref_step;
		if (app.runtime.speed_ref > app.setup.speed_ref_max) {
			app.runtime.speed_ref = app.setup.speed_ref_max;
		}
		break;
	case 'd':
		app.runtime.speed_ref -= app.setup.speed_ref_step;
		if (app.runtime.speed_ref < -app.setup.speed_ref_max) {
			app.runtime.speed_ref = -app.setup.speed_ref_max;
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
		app.runtime.Idq_ref.q -= 0.1F;
		if (app.runtime.Idq_ref.q < 0.0F) {
			app.runtime.Idq_ref.q = 0.0F;
		}
		break;
	case 'x':
		app.runtime.Idq_ref.q += 0.1F;
		if (app.runtime.Idq_ref.q > app.setup.iq_ref_max) {
			app.runtime.Idq_ref.q = app.setup.iq_ref_max;
		}
		break;
	case 'l':
	case 'L':
		toggle_open_loop_mode(app);
		break;
	case 'j':
	case 'J':
		app.runtime.omega_ol -= app.setup.open_loop_speed_step;
		printk("open-loop speed = %.2f rad/s\n", (double)app.runtime.omega_ol);
		break;
	case 'k':
	case 'K':
		app.runtime.omega_ol += app.setup.open_loop_speed_step;
		printk("open-loop speed = %.2f rad/s\n", (double)app.runtime.omega_ol);
		break;
	case 'n':
	case 'N':
		app.runtime.vq_ol -= app.setup.open_loop_vq_step;
		printk("open-loop vq = %.2f V\n", (double)app.runtime.vq_ol);
		break;
	case 'b':
	case 'B':
		app.runtime.vq_ol += app.setup.open_loop_vq_step;
		printk("open-loop vq = %.2f V\n", (double)app.runtime.vq_ol);
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
		app.runtime.memory_print = !app.runtime.memory_print;
		break;
	case 'c':
	case 'C':
		shield.position.setAbzSpeedDecimation(app.runtime.speed_decimation + 1U);
		break;
	case 'v':
	case 'V':
		shield.position.setAbzSpeedDecimation(app.runtime.speed_decimation - 1U);
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
	if (!app.runtime.memory_print) {
		printk("%7.2f:", (double)app.runtime.V_high);
		printk("%7.2f:", (double)app.runtime.Iq_max);
		printk("%7.2f:", (double)app.runtime.speed_ref);
		printk("%7.2f:", (double)app.runtime.w_meas);
		printk("%7.2f:", (double)app.runtime.I1_offset);
		printk("%7d:", control_state);
		printk("%7u:", app.runtime.encoder_count);
		printk("%7ld:", (long)app.runtime.encoder_delta_count);
		printk("%7.2f:", (double)shield.position.getElectricalOffset());
		printk("%7d:", shield.position.getDirectionSign());
		printk("%7d:", app.runtime.open_loop_mode ? 1 : 0);
		printk("%7.2f:", (double)app.runtime.angle_filtered);
		printk("%7.2f:", (double)app.runtime.theta_ol);
		printk("%7.2f:", (double)app.runtime.omega_ol);
		printk("%7.2f:", (double)app.runtime.Idq_ref.q);
		printk("%7.2f\n", (double)app.runtime.vq_ol);
	} else {
		app.runtime.k_app_idx = (app.runtime.k_app_idx + 1U) % SCOPE_SIZE;
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 0));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 1));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 2));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 3));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 4));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 5));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 6));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 7));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 8));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 9));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 10));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 11));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 12));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 13));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 14));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 15));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 16));
		printk("%.2f:", (double)scope.get_channel_value(app.runtime.k_app_idx, 17));
		printk("%.2f", (double)scope.get_channel_value(app.runtime.k_app_idx, 18));
		printk("\n");
	}

	if (app.runtime.is_downloading) {
		dump_scope_datas(scope);
		app.runtime.is_downloading = false;
	}

	switch (control_state) {
	case OFFSET_ST:
		if (app.runtime.counter_time > (uint32_t)app.setup.nb_offset) {
			spin.led.turnOff();
			app.runtime.I1_offset = -app.runtime.tmpI1_offset / app.setup.nb_offset;
			app.runtime.I2_offset = -app.runtime.tmpI2_offset / app.setup.nb_offset;
			app.runtime.position_data_valid = false;
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) &&
			app.runtime.position_sensor_initialized &&
			app.runtime.position_data_valid &&
			(app.runtime.V_high_filtered > app.setup.v_high_min)) {
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
			app.runtime.error_counter = 0U;
			control_state = IDLE_ST;
		}
		break;
	}

	task.suspendBackgroundMs(250);
}

void loop_critical_task()
{
	app.runtime.counter_time++;

	retrieve_analog_data(app, control_state == OFFSET_ST);
	update_position_and_speed(app);

	if ((control_state == POWER_ST) && !app.runtime.position_data_valid) {
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
		if (app.runtime.open_loop_mode) {
			control_open_loop(app);
		} else {
			control_speed(app);
		}
		compute_duties(app);
		apply_duties(app);
		start_pwms_if_needed(app);
		break;
	}

	if (app.runtime.counter_time % app.setup.decimation == 0U) {
		update_scope_mirrors();
		scope.acquire();
	}
}

int main(void)
{
	setup_routine();
	return 0;
}
