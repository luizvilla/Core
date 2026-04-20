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

/* --------------OWNTECH APIs---------------------------------- */

#include "ScopeMimicry.h"
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h"
#include "zephyr/console/console.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();
void adjust_electrical_offset(float32_t delta);
void toggle_open_loop_mode();
void adjust_speed_loop_kp(float32_t delta);
void adjust_speed_loop_ki(float32_t delta);

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical */
void loop_critical_task();
void application_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */
static const float32_t AC_CURRENT_LIMIT = 4.0;
static const float32_t DC_CURRENT_LIMIT = 4.0;

/* Control timing and startup thresholds. */
static const float32_t MIN_DC_VOLTAGE = 30.0F;
/* Used as a threshold to start POWER mode */
static const float32_t V_HIGH_MIN = 5.0;
static const float32_t Ts = 100.e-6F;
static const uint32_t control_task_period = (uint32_t)(Ts * 1.e6F);
/* The speed loop is intentionally 10x slower than the current loop. */
static const uint32_t speed_loop_decimation = 10;
static const float32_t Ts_speed = Ts * speed_loop_decimation;
static const float32_t ELECTRICAL_OFFSET_STEP = PI / 180.0F;
static const float32_t OPEN_LOOP_SPEED_STEP = 1.0F;
static const float32_t OPEN_LOOP_VQ_STEP = 0.1F;
static const float32_t SPEED_KP_STEP = 0.001F;
static const float32_t SPEED_KI_STEP = 0.05F;
static float32_t angle_filtered;
static float32_t w_meas;

/* Power LEG measures */
static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;
static const float32_t NB_OFFSET = 2000.0;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V12_value;

/* DC measures */
static float32_t I_high;
static float32_t V_high;

/* Position sensor */
static uint32_t encoder_count;
static uint32_t encoder_count_prev;
static int32_t encoder_delta_count;
static float32_t encoder_mech_angle;
static float32_t encoder_elec_angle;
static float32_t encoder_mech_speed;
static float32_t encoder_elec_speed;
static position_sensor_type_t active_position_type;
static bool position_sensor_initialized;
static bool position_data_valid;

/* Three phase system and Park DQ Frame (dqo) */
static three_phase_t Vabc;
static three_phase_t duty_abc;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;
static float32_t angle_4_control;

/* Variables mirrored to ScopeMimicry for logging and tuning. */
static three_phase_t Iabc_ref;
static float32_t duty_a, duty_b;
static float32_t Ia_ref;
static float32_t Ib_ref;
static float32_t Va;
static float32_t Iq_meas;
static float32_t Iq_ref;
static float32_t Id_ref;
static float32_t Iq_max;
static float32_t Vd, Vq;
static float32_t speed_ref;
static float32_t speed_ref_print;
static float32_t speed_meas_print;
static float32_t electrical_offset_print;
static float32_t open_loop_mode_print;
static float32_t theta_ol_print;
static float32_t omega_ol_print;
static float32_t vq_ol_print;
static float32_t iq_ref_from_speed;
static float32_t encoder_count_f;
static float32_t encoder_delta_count_f;

/* Speed command and limits. The outer loop generates the q-axis current reference. */
static const float32_t SPEED_REF_STEP = 1.0F;
static const float32_t SPEED_REF_MAX = 300.0F;
static const float32_t IQ_REF_MAX = 4.0F;

/* Filters used on bus voltage and estimated electrical speed. */
static LowPassFirstOrderFilter vHigh_filter =
						controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static LowPassFirstOrderFilter w_mes_filter =
						controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static float32_t V_high_filtered;
static float32_t inverse_Vhigh;

/* Inner current regulators and outer speed regulator. */
static float32_t Kp = 30 * 0.035;
static float32_t Ti = 0.002029;
static float32_t Td = 0.0F;
static float32_t N = 1.0;


/* Coefficient 0.4 comes from Va_max =  (α_max - 0.5) * Udc     */
static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.4;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.4;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, Td, N,
										lower_bound, upper_bound);

static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, Td, N,
										lower_bound, upper_bound);

static float32_t speed_Kp = 0.01F;
static float32_t speed_Ti = 0.1F;
static float32_t speed_Ki = 0.1F;
static Pid pi_speed = controlLibFactory.pid(Ts_speed, speed_Kp, speed_Ti, 0.0F, 1.0F,
											-IQ_REF_MAX, IQ_REF_MAX);
static uint8_t speed_decimation = 10;


/* Scope decimation only affects logging, not control execution. */
const static uint32_t decimation = 20;
static uint32_t counter_time;
float32_t counter_time_f;
uint8_t received_serial_char;

/* List of possible modes for the OwnTech power shield */
enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

/* List of possible control states */
enum control_state_mode {
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

enum control_state_mode control_state;
static float32_t control_state_f;

static uint16_t error_counter;
static bool pwm_enable;
uint8_t asked_mode = IDLEMODE;
static bool open_loop_mode;
static float32_t theta_ol;
static float32_t omega_ol;
static float32_t vq_ol;

/* ScopeMimicry capture state and helpers. */

const uint16_t SCOPE_SIZE = 512;
uint16_t k_app_idx;
ScopeMimicry scope(SCOPE_SIZE, 19);
static bool is_downloading;
static bool memory_print;

bool mytrigger()
{
	return (control_state == POWER_ST);
}

void dump_scope_datas(ScopeMimicry &scope) {
	printk("begin record\n");
	scope.reset_dump();
	while (scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

/**
 * Reset estimator, filters and controller states.
 */
void init_filt_and_reg(void)
{
	vHigh_filter.reset(V_HIGH_MIN);
	pi_d.reset();
	pi_q.reset();
	pi_speed.reset();
	error_counter = 0;
}

static int32_t normalize_encoder_delta(uint32_t current_count,
									   uint32_t previous_count,
									   uint32_t counts_per_revolution)
{
	int32_t delta = (int32_t)current_count - (int32_t)previous_count;

	if (counts_per_revolution == 0U) {
		return delta;
	}

	if (delta > ((int32_t)counts_per_revolution / 2)) {
		delta -= (int32_t)counts_per_revolution;
	} else if (delta < -((int32_t)counts_per_revolution / 2)) {
		delta += (int32_t)counts_per_revolution;
	}

	return delta;
}


/**
 * Retrieve the latest sensor values and update filtered quantities.
 */
inline void retrieve_analog_datas()
{
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data + I1_offset;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data + I2_offset;
	}

	if (control_state == OFFSET_ST && counter_time < NB_OFFSET) {
		tmpI1_offset += I1_low_value;
		tmpI2_offset += I2_low_value;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		/* Sign is negative because of the way hardware sensor is routed */
		I_high = -meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	/* Vhigh measurement gets additional filtering */
	V_high_filtered = vHigh_filter.calculateWithReturn(V_high);

	V12_value = V1_low_value - V2_low_value;
}

/**
 * Update the shield position estimator and retrieve angle/speed data.
 */
inline void get_position_and_speed()
{
	position_data_valid = false;
	if (!position_sensor_initialized) {
		return;
	}

	if (!shield.position.update(Ts)) {
		return;
	}

	if (active_position_type == ABZ_TYPE) {
		uint32_t counts_per_revolution = shield.position.getCountsPerRevolution();
		encoder_count = shield.position.getIncrementalEncoderValue();
		encoder_delta_count = normalize_encoder_delta(encoder_count,
													  encoder_count_prev,
													  counts_per_revolution);
		encoder_count_prev = encoder_count;
	} else {
		encoder_count = 0U;
		encoder_delta_count = 0;
	}

	encoder_mech_angle = shield.position.getMechanicalAngle();
	encoder_elec_angle = shield.position.getElectricalAngle();
	encoder_mech_speed = shield.position.getMechanicalSpeed();
	encoder_elec_speed = shield.position.getElectricalSpeed();

	angle_filtered = encoder_elec_angle;
	w_meas = w_mes_filter.calculateWithReturn(encoder_elec_speed);
	position_data_valid = true;
}

/**
 * Count repeated overcurrent events and latch the error state if needed.
 */
inline void overcurrent_mngt()
{
	if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT ||
	    I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT ||
	    I_high > DC_CURRENT_LIMIT) {
		error_counter++;
	}
	if (error_counter > 1000) {
		control_state = ERROR_ST;
	}
}

/**
 * Stop PWM and clear controller state when leaving closed-loop operation.
 */
inline void stop_pwm_and_reset_states_ifnot()
{
	if (pwm_enable == true) {
		shield.power.stop(ALL);
		/* Reset filters and pid */
		init_filt_and_reg();
		pwm_enable = false;
	}

	theta_ol = 0.0F;
}

/**
 * Restart current-sensor offset calibration from a clean stopped state.
 */
inline void restart_offset_calibration()
{
	stop_pwm_and_reset_states_ifnot();
	counter_time = 0;
	encoder_count_prev = 0U;
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	asked_mode = IDLEMODE;
	control_state = OFFSET_ST;
	spin.led.turnOn();
}

/**
 * Run cascaded control:
 * - the speed PI executes every `speed_loop_decimation` current-loop ticks
 * - the current PIs execute every critical-task period
 */
inline void control_speed()
{
	// theta_ol = ot_modulo_2pi(theta_ol + omega_ol * Ts);
	// angle_4_control = theta_ol;


	angle_4_control = angle_filtered;
	/* Hold the previous q-axis current reference between speed-loop updates. */
	// if ((counter_time % speed_loop_decimation) == 0U) {
	// 	iq_ref_from_speed = pi_speed.calculateWithReturn(speed_ref, w_meas);
	// }


    // Idq_ref.q = iq_ref_from_speed;

	/* Saturation */
	if (Idq_ref.q > Iq_max) {
		Idq_ref.q = Iq_max;
	}
	if (Idq_ref.q < -Iq_max) {
		Idq_ref.q = -Iq_max;
	}

	Idq_ref.d = 0.0F;
	Iabc.a = I1_low_value;
	Iabc.b = I2_low_value;
	Iabc.c = -(Iabc.a + Iabc.b);

	Idq = Transform::to_dqo(Iabc, angle_4_control);
	Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
	Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
	Vdq.o = 0.0F;

	Vabc = Transform::to_threephase(Vdq, angle_4_control);
}

/**
 * Run the inverter from a synthetic rotating electrical angle while still
 * updating the encoder path for observation.
 */
inline void control_open_loop()
{
	theta_ol = ot_modulo_2pi(theta_ol + omega_ol * Ts);
	angle_4_control = theta_ol;

	Iabc.a = I1_low_value;
	Iabc.b = I2_low_value;
	Iabc.c = -(Iabc.a + Iabc.b);
	Idq = Transform::to_dqo(Iabc, angle_4_control);

	Idq_ref.d = 0.0F;
	Idq_ref.q = 0.0F;
	Vdq.d = 0.0F;
	Vdq.q = vq_ol;
	Vdq.o = 0.0F;

	Vabc = Transform::to_threephase(Vdq, angle_4_control);
}

/**
 * Convert commanded phase voltages into PWM duty cycles.
 */
inline void compute_duties()
{
	inverse_Vhigh = 1.0 / MIN_DC_VOLTAGE;
	duty_abc.a = (Vabc.a * inverse_Vhigh + 0.5);
	duty_abc.b = (Vabc.b * inverse_Vhigh + 0.5);
	duty_abc.c = (Vabc.c * inverse_Vhigh + 0.5);
}

/**
 * Apply the computed duty cycles to the three power legs.
 */
inline void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_abc.a);
	shield.power.setDutyCycle(LEG2, duty_abc.b);
	shield.power.setDutyCycle(LEG3, duty_abc.c);
}

/**
 * Start the PWM outputs once when entering power mode.
 */
void start_pwms_ifnot()
{
	if (!pwm_enable) {
		pwm_enable = true;
		shield.power.start(ALL);
	}
}

/**
 * Initialize runtime variables before the first control activation.
 */
void init_variables()
{
	/* Time counter */
	counter_time = 0;
	/* Measurements variables */
	I1_low_value = 0.0F;
	I2_low_value = 0.0F;
	I_high = 0.0F;
	V_high = 0.0F;
	/* Offset variables */
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	/* State view of the pwm */
	pwm_enable = false;
	/* Idle or power mode*/
	asked_mode = IDLEMODE;
	/* We begin to measure the current offset before all */
	control_state = IDLE_ST;
	Iq_max = IQ_REF_MAX;
	speed_ref = 0.0F;
	speed_ref_print = 0.0F;
	speed_meas_print = 0.0F;
	electrical_offset_print = 0.0F;
	open_loop_mode = false;
	open_loop_mode_print = 0.0F;
	theta_ol = 0.0F;
	theta_ol_print = 0.0F;
	omega_ol = 10.0F;
	omega_ol_print = omega_ol;
	vq_ol = 0.5F;
	vq_ol_print = vq_ol;
	iq_ref_from_speed = 0.0F;
	encoder_count = 0U;
	encoder_delta_count = 0;
	encoder_mech_angle = 0.0F;
	encoder_elec_angle = 0.0F;
	encoder_mech_speed = 0.0F;
	encoder_elec_speed = 0.0F;
	position_data_valid = false;
	restart_offset_calibration();
}
/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * In this setup routine :
 *  - Power shield is initialized
 * 		- Shield is set in Buck Mode.
 * 		- Default sensors are activated
 * 		- Default position sensor is initialized from app.overlay
 *  - ScopeMimicry is initialized
 * 	- VHigh filter and PIDs are initialized
 * 	- LED is turned on.
 *  - Tasks are initialized and started
 */
void setup_routine()
{
	/* Setup the hardware first */
	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();
	position_sensor_initialized = shield.position.initDefault();
	if (!position_sensor_initialized) {
		printk("ERROR: failed to initialize the default position sensor from app.overlay.\n");
	}
	active_position_type = shield.position.getActiveSensorType();

	/* Scope configuration */
	scope.connectChannel(V12_value, "V12_value");           /* 0 */
	scope.connectChannel(Vq, "Vq");                         /* 1 */
	scope.connectChannel(Vd, "Vd");                         /* 2 */
	scope.connectChannel(I1_low_value, "I1_low_value");     /* 3 */
	scope.connectChannel(I2_low_value, "I2_low_value");     /* 4 */
	scope.connectChannel(I_high, "I_high_value");     	    /* 5 */
	scope.connectChannel(Iq_meas, "Iq_meas");               /* 6 */
	scope.connectChannel(speed_meas_print, "speed_meas");   /* 7 */
	scope.connectChannel(speed_ref_print, "speed_ref");     /* 8 */
	scope.connectChannel(encoder_elec_angle, "encoder_angle"); /* 9 */
	scope.connectChannel(angle_filtered, "angle_filtered"); /* 10 */
	scope.connectChannel(control_state_f, "control_state"); /* 11 */
	scope.connectChannel(electrical_offset_print, "electrical_offset"); /* 12 */
	scope.connectChannel(open_loop_mode_print, "open_loop_mode"); /* 13 */
	scope.connectChannel(theta_ol_print, "theta_ol"); /* 14 */
	scope.connectChannel(omega_ol_print, "omega_ol"); /* 15 */
	scope.connectChannel(vq_ol_print, "vq_ol"); /* 16 */
	scope.connectChannel(Idq_ref.d, "Id_ref"); /* 17 */
	scope.connectChannel(Idq_ref.q, "Iq_ref"); /* 18 */
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();

	/* Initialize values */
	init_filt_and_reg();
	init_variables();
	spin.led.turnOn();


	/* Declare tasks */
	uint32_t background_task_number =
					task.createBackground(loop_background_task);

	uint32_t app_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, control_task_period);

	/* Finally, start tasks */
	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * Poll USB serial commands:
 * - P / I: enter power mode or idle mode
 * - U / D: increase or decrease the speed reference
 * - [ / ]: decrease or increase electrical offset by 1 degree
 * - L: toggle open-loop mode
 * - J / K: decrease or increase open-loop electrical speed
 * - N / B: decrease or increase open-loop q-axis voltage
 * - X: reset the open-loop electrical angle
 * - T / Y: decrease or increase speed-loop Kp
 * - G / H: decrease or increase speed-loop Ki
 * - O: restart current-offset calibration
 * - R / Q / M: control ScopeMimicry data capture and replay
 */
void loop_background_task()
{
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'p':
		printk("power asked");
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'i':
		printk("idle asked");
		asked_mode = IDLEMODE;
		speed_ref = 0.0F;
		break;
	case 'o':
		printk("offset recalibration asked");
		restart_offset_calibration();
		break;
	case 'r':
		is_downloading = true;
		break;
	case 'u':
		speed_ref += SPEED_REF_STEP;
		if (speed_ref > SPEED_REF_MAX) {
			speed_ref = SPEED_REF_MAX;
		}
		break;
	case 'd':
		speed_ref -= SPEED_REF_STEP;
		if (speed_ref < -SPEED_REF_MAX) {
			speed_ref = -SPEED_REF_MAX;
		}
		break;
	case '[':
		adjust_electrical_offset(-ELECTRICAL_OFFSET_STEP);
		scope.start();
		break;
	case ']':
		adjust_electrical_offset(ELECTRICAL_OFFSET_STEP);
		scope.start();
		break;
	case 'z':
		Idq_ref.q -= 0.1F;
		if (Idq_ref.q < 0.0) {
			Idq_ref.q = 0.0;
		}
		break;
	case 'x':
		Idq_ref.q += 0.1F;
		if (Idq_ref.q > IQ_REF_MAX) {
			Idq_ref.q = IQ_REF_MAX;
		}
		break;
	case 'l':
	case 'L':
		toggle_open_loop_mode();
		break;
	case 'j':
	case 'J':
		omega_ol -= OPEN_LOOP_SPEED_STEP;
		printk("open-loop speed = %.2f rad/s\n", (double)omega_ol);
		break;
	case 'k':
	case 'K':
		omega_ol += OPEN_LOOP_SPEED_STEP;
		printk("open-loop speed = %.2f rad/s\n", (double)omega_ol);
		break;
	case 'n':
	case 'N':
		vq_ol -= OPEN_LOOP_VQ_STEP;
		printk("open-loop vq = %.2f V\n", (double)vq_ol);
		break;
	case 'b':
	case 'B':
		vq_ol += OPEN_LOOP_VQ_STEP;
		printk("open-loop vq = %.2f V\n", (double)vq_ol);
		break;
	// case 'x':
	// case 'X':
	// 	theta_ol = 0.0F;
	// 	printk("open-loop angle reset\n");
	// 	break;
	case 't':
	case 'T':
		adjust_speed_loop_kp(-SPEED_KP_STEP);
		break;
	case 'y':
	case 'Y':
		adjust_speed_loop_kp(SPEED_KP_STEP);
		break;
	case 'g':
	case 'G':
		adjust_speed_loop_ki(-SPEED_KI_STEP);
		break;
	case 'h':
	case 'H':
		adjust_speed_loop_ki(SPEED_KI_STEP);
		break;
	case 'm':
		/* To print scope datas in ownplot as soon as possible */
		memory_print = !memory_print;
		break;
	case 'c':
	case 'C':
		shield.position.setAbzSpeedDecimation(speed_decimation + 1);
		break;
	case 'v':
	case 'V':
		shield.position.setAbzSpeedDecimation(speed_decimation - 1);
		break;
	case 's':
	case 'S':
		/* invert direction sign */
		shield.position.setDirectionSign(-shield.position.getDirectionSign());
		break;		


		case 'q':
		/* Relaunch scope acquisition */
		scope.start();
		break;
	}
}

void adjust_electrical_offset(float32_t delta)
{
	float32_t current_offset = shield.position.getElectricalOffset();
	float32_t new_offset = ot_modulo_2pi(current_offset + delta);
	shield.position.setElectricalOffset(new_offset);

	printk("electrical offset = %.4f rad (%.1f deg)\n",
		   (double)new_offset,
		   (double)(new_offset * 180.0F / PI));
}

void toggle_open_loop_mode()
{
	open_loop_mode = !open_loop_mode;
	theta_ol = encoder_elec_angle;
	init_filt_and_reg();

	printk("open-loop mode %s, theta_ol = %.4f rad, omega_ol = %.2f rad/s, vq_ol = %.2f V\n",
		   open_loop_mode ? "enabled" : "disabled",
		   (double)theta_ol,
		   (double)omega_ol,
		   (double)vq_ol);
}

void adjust_speed_loop_kp(float32_t delta)
{
	speed_Kp += delta;
	if (speed_Kp < 1.0e-6F) {
		speed_Kp = 1.0e-6F;
	}
	pi_speed.setKp(speed_Kp);
	pi_speed.setKi(speed_Ki);
	pi_speed.reset();
	speed_Ti = pi_speed.getTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)speed_Kp,
		   (double)speed_Ki,
		   (double)speed_Ti);
}

void adjust_speed_loop_ki(float32_t delta)
{
	speed_Ki += delta;
	if (speed_Ki < 1.0e-6F) {
		speed_Ki = 1.0e-6F;
	}
	pi_speed.setKi(speed_Ki);
	pi_speed.reset();
	speed_Ti = pi_speed.getTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)speed_Kp,
		   (double)speed_Ki,
		   (double)speed_Ti);
}

/**
 * Stream status data over USB serial and handle scope dumps.
 */
void application_task()
{
	if (!memory_print) {
		printk("%7.2f:", V_high);									/* A */
		printk("%7.2f:", Iq_max);									/* B */
		printk("%7.2f:", speed_ref);								/* C */
		printk("%7.2f:", w_meas);									/* D */
		printk("%7.2f:", I1_offset);								/* E */
		printk("%7d:", control_state);								/* F */
		printk("%7u:", encoder_count);								/* G */
		printk("%7ld:", (long)encoder_delta_count);					/* H */
		printk("%7.2f:", shield.position.getElectricalOffset());	/* I */
		printk("%7.2f:", shield.position.getDirectionSign());		/* J */
		printk("%7.0f:", open_loop_mode ? 1.0 : 0.0);				/* K */
		printk("%7.2f:", angle_filtered);							/* L */		
		printk("%7.2f:", theta_ol);									/* M */		
		printk("%7.2f:", omega_ol);									/* N */
		printk("%7.2f:", Idq_ref.q);								/* O */	
		printk("%7.2f\n", vq_ol);									/* P */
	} else {
		/* Replay the scope buffer continuously over serial for live plotting tools.
		 */
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.2f:", scope.get_channel_value(k_app_idx, 0));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 1));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 2));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 3));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 4));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 5));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 6));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 7));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 8));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 9));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 10));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 11));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 12));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 13));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 14));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 15));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 16));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 17));
		printk("%.2f", scope.get_channel_value(k_app_idx, 18));
		printk("\n");
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}
	switch (control_state) {
	case OFFSET_ST:
		if (counter_time > (uint32_t)NB_OFFSET) {
			spin.led.turnOff();
			I1_offset = -tmpI1_offset / NB_OFFSET;
			I2_offset = -tmpI2_offset / NB_OFFSET;
			position_data_valid = false;
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) &&
			position_sensor_initialized &&
			position_data_valid &&
			(V_high_filtered > V_HIGH_MIN)) {
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
			error_counter = 0;
			control_state = IDLE_ST;
		}
		break;
	}

	task.suspendBackgroundMs(250);
}


/**
 * Critical 10 kHz task:
 * - acquire measurements and encoder state
 * - update the cascaded control loops
 * - apply PWM duties
 */
void loop_critical_task()
{
	counter_time++;

	retrieve_analog_datas();

	get_position_and_speed();

	if ((control_state == POWER_ST) && !position_data_valid) {
		control_state = ERROR_ST;
	}

	overcurrent_mngt();

	switch (control_state) {
	case OFFSET_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case IDLE_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case ERROR_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case POWER_ST:
		/* Closed-loop speed/current control runs only in power mode. */
		if (open_loop_mode) {
			control_open_loop();
		} else {
			control_speed();
		}
		compute_duties();
		apply_duties();
		start_pwms_ifnot();
		break;
	}

	/* Decimate scope acquisition to keep logging bandwidth reasonable. */
	if (counter_time % decimation == 0) {
		encoder_count_f = (float32_t)encoder_count;
		encoder_delta_count_f = (float32_t)encoder_delta_count;
		Va = Vabc.a;
			duty_a = duty_abc.a;
			duty_b = duty_abc.b;
			Iq_ref = Idq_ref.q;
			Id_ref = Idq_ref.d;
			Iq_meas = Idq.q;
			Vd = Vdq.d;
			Vq = Vdq.q;
			speed_ref_print = speed_ref;
			speed_meas_print = w_meas;
			electrical_offset_print = shield.position.getElectricalOffset();
			open_loop_mode_print = open_loop_mode ? 1.0F : 0.0F;
			theta_ol_print = theta_ol;
			omega_ol_print = omega_ol;
			vq_ol_print = vq_ol;
			Iabc_ref = Transform::to_threephase(Idq_ref, angle_4_control);
		Ia_ref = Iabc_ref.a;
		Ib_ref = Iabc_ref.b;
		counter_time_f = (float32_t)counter_time;
		control_state_f = control_state;
		scope.acquire();
	}
}

int main(void)
{
	setup_routine();

	return 0;
}
