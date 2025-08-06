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
 * @brief  This file is an example of a Field Oriented Control for
 *         OwnTech OwnVerter board.
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
#include "CommunicationAPI.h"
#include "ShieldAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h"
#include "zephyr/console/console.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical */
void loop_critical_task();
void application_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

static const float32_t AC_CURRENT_LIMIT = 20.0;
static const float32_t DC_CURRENT_LIMIT = 20.0;

/* Used in the control algorithm */
static const float32_t MIN_DC_VOLTAGE = 30.0F;
/* Used as a threshold to start POWER mode */
static const float32_t V_HIGH_MIN = 5.0;
static const float32_t Ts = 100.e-6F;
static const uint32_t control_task_period = (uint32_t)(Ts * 1.e6F);

/* Variables to compute Sensor offset */
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;

/* Power LEG measures */
static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I1_offset;
static float32_t I2_offset;
static const float32_t NB_OFFSET = 2000.0;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V12_value;

/* DC measures */
static float32_t I_high;
static float32_t V_high;

/* Three phase system and Park DQ Frame (dqo) */
/* Three phase system and Park DQ Frame (dqo) */
static three_phase_t Vabc;
static three_phase_t duty_abc;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;
static float32_t angle_4_control;

/* We only make torque control. */
static float32_t manual_Iq_ref;

uint8_t buffer_tx[6];
uint8_t buffer_rx[6];

/* Communication frames structures */
typedef struct {
	float32_t angle;
	uint8_t  status:8;
	uint8_t  nodata:8;
} __packed angle_frame_t;

typedef struct {
	float32_t current;
	uint8_t  status:8;
	uint8_t  nodata:8;
} __packed current_frame_t;


current_frame_t data_current_2_send;
angle_frame_t data_angle_received;

/* Variables used to get static value for ScopeMimicry */
static three_phase_t Iabc_ref;
static float32_t duty_a, duty_b;
static float32_t Ia_ref;
static float32_t Ib_ref;
static float32_t Va;
static float32_t Iq_meas;
static float32_t Iq_ref;
static float32_t Iq_max;
static float32_t Vd, Vq;
static float32_t angle_ref;


/**
 * Low Pass Filters Init
 */
static LowPassFirstOrderFilter vHigh_filter =
						controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static float32_t V_high_filtered;
static float32_t inverse_Vhigh;

/**
 *  PID
 */

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


/* Decimation is used to limit the rate of measurement plotted in ScopeMimicry*/
const static uint32_t decimation = 10;
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

/**
 *  Definition of variables and functions for plotting real time values
 *  using ScopeMimicry.
 */

const uint16_t SCOPE_SIZE = 512;
uint16_t k_app_idx;
ScopeMimicry scope(SCOPE_SIZE, 10);
static bool is_downloading;
static bool memory_print;

void reception_function(void)
{
	/* Here we are the communication master, so when we the slave send something
	we receive the current frame. */
	data_angle_received = *(angle_frame_t *) buffer_rx;
	if (data_angle_received.angle != -1)
	{
		angle_4_control = data_angle_received.angle;
	}
	else
	{
		control_state = IDLE_ST;
	}
	communication.rs485.startTransmission();
}

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
 * VHigh Filter and PID init function
 */
void init_filt_and_reg(void)
{
	vHigh_filter.reset(V_HIGH_MIN);
	pi_d.reset();
	pi_q.reset();
	error_counter = 0;
}

/**
 *  Function that retrieves necessary measurements from power shield sensors.
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
 * Handles current limits and switch to Error state if limits exceeded.
 */
inline void overcurrent_mngt()
{
	if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT ||
	    I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT ||
	    I_high > DC_CURRENT_LIMIT) {
		error_counter++;
	}
	if (error_counter > 2) {
		control_state = ERROR_ST;
	}
}

/**
 * Performs Torque control using Field Oriented Control algorithm
 */
inline void control_torque()
{

	Idq_ref.q = manual_Iq_ref;

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
 * Stops PWM and reset filter and PID states
 */
inline void stop_pwm_and_reset_states_ifnot()
{
	if (pwm_enable == true) {
		shield.power.stop(ALL);
		/* Reset filters and pid */
		init_filt_and_reg();
		pwm_enable = false;
	}
}

/**
 * Helper function that computes duty cycles from ABC frame.
 */
inline void compute_duties()
{
	inverse_Vhigh = 1.0 / MIN_DC_VOLTAGE;
	duty_abc.a = (Vabc.a * inverse_Vhigh + 0.5);
	duty_abc.b = (Vabc.b * inverse_Vhigh + 0.5);
	duty_abc.c = (Vabc.c * inverse_Vhigh + 0.5);
}

/**
 * Helper function that set the duty cycles to the power shield.
 */
inline void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_abc.a);
	shield.power.setDutyCycle(LEG2, duty_abc.b);
	shield.power.setDutyCycle(LEG3, duty_abc.c);
}

/**
 * Helper function to start the power shield PWMs
 */
void start_pwms_ifnot()
{
	if (!pwm_enable) {
		pwm_enable = true;
		shield.power.start(ALL);
	}
}

/**
 * Setter function for required variables
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
	/* State view of the pwm */
	pwm_enable = false;
	/* Idle or power mode*/
	asked_mode = IDLEMODE;
	/* We begin to measure the current offset before all */
	control_state = OFFSET_ST;


	Iq_max = 2.0;
	manual_Iq_ref = 0.0F;
}
/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * In this setup routine :
 *  - Power shield is initialized
 * 		- Shield is set in Buck Mode.
 * 		- Default sensors are activated
 *  - ScopeMimicry is initialized
 * 	- VHigh filter and PIDs are initialized
 * 	- LED is turned on.
 *  - Tasks are initialized and started
 */
void setup_routine()
{
	/* Setup the hardware first */
	shield.power.initBuck(ALL);
	communication.sync.initSlave();
	communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_tx), reception_function, SPEED_10M);
	shield.sensors.enableDefaultPowerverterSensors();

	/* Scope configuration */
	scope.connectChannel(V12_value, "V12_value");           /* 0 */
	scope.connectChannel(Vq, "Vq");                         /* 1 */
	scope.connectChannel(Vd, "Vd");                         /* 2 */
	scope.connectChannel(I1_low_value, "I1_low_value");     /* 3 */
	scope.connectChannel(I2_low_value, "I2_low_value");     /* 4 */
	scope.connectChannel(I_high, "I_high_value");     	    /* 5 */
	scope.connectChannel(Iq_meas, "Iq_meas");               /* 6 */
	scope.connectChannel(Ib_ref, "Ib_ref");                 /* 7 */
	scope.connectChannel(Ia_ref, "Ia_ref");                 /* 8 */
	scope.connectChannel(angle_ref, "angle_ref");           /* 9 */
	scope.connectChannel(control_state_f, "control_state"); /* 10 */
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
	task.createCritical(loop_critical_task, control_task_period, source_hrtim);

	/* Finally, start tasks */
	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This background task retrieve user inputs to control the OwnVerter:
 * - P and I keys respectively Power ON and Power OFF the inverter
 * - U and D keys respectively Increase and Decrease Iq reference.
 * - R Q and M keys are used to control ScopeMimicry data retrieval.
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
		break;
	case 'r':
		is_downloading = true;
	case 'u':
		manual_Iq_ref += 0.1;
		break;
	case 'd':
		manual_Iq_ref -= 0.1F;
		break;
	case 'm':
		/* To print scope datas in ownplot as soon as possible */
		memory_print = !memory_print;
		break;
	case 'q':
		/* Relaunch scope acquisition */
		scope.start();
		break;
	}
}

/**
 * This application task sends data over USB Serial.
 */
void application_task()
{
	if (!memory_print) {
		printk("%7.2f:", V_high);
		printk("%7.2f:", Iq_max);
		printk("%7.2f:", manual_Iq_ref);
		printk("%7.2f:", I1_offset);
		printk("%7d:\r\n", control_state);

	} else {
		/* If memory_print is true then we plot scope datas in an infinite loop
		 * This can be used with ownplot if you have not python script installed
		 * to plot downloaded data using dump_scope_datas().
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
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) && (V_high_filtered > V_HIGH_MIN)) {
			control_state = POWER_ST;
		}
		break;

	case POWER_ST:
		if (asked_mode == IDLEMODE) {
			control_state = IDLE_ST;
		}
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
 * This is the critical task that runs at 10kHz.
 * It performs the Field Oriented Control.
 */
void loop_critical_task()
{
	counter_time++;

	retrieve_analog_datas();

	overcurrent_mngt();

	switch (control_state) {
	case OFFSET_ST:
		stop_pwm_and_reset_states_ifnot();
		data_current_2_send.current = -1;
		data_current_2_send.status = OFFSET_ST;
		memcpy(buffer_tx, &data_current_2_send, sizeof(data_current_2_send));
		break;
	case IDLE_ST:
		stop_pwm_and_reset_states_ifnot();
		data_current_2_send.current = -1;
		data_current_2_send.status = IDLE_ST;
		memcpy(buffer_tx, &data_current_2_send, sizeof(data_current_2_send));
		break;
	case ERROR_ST:
		stop_pwm_and_reset_states_ifnot();
		data_current_2_send.current = -1;
		data_current_2_send.status = ERROR_ST;
		memcpy(buffer_tx, &data_current_2_send, sizeof(data_current_2_send));
		break;
	case POWER_ST:
		/* Control loop is executed here */
		control_torque();
		compute_duties();
		apply_duties();
		start_pwms_ifnot();

		/* send angle reference to slave */
		data_current_2_send.current = Idq.q;
		data_current_2_send.status = POWER_ST;
		memcpy(buffer_tx, &data_current_2_send, sizeof(data_current_2_send));
		break;
	}

	/* Decimation is used to reduce rate of plotting in ScopeMimicry */
	if (counter_time % decimation == 0) {
		Va = Vabc.a;
		duty_a = duty_abc.a;
		duty_b = duty_abc.b;
		Iq_ref = Idq_ref.q;
		Iq_meas = Idq.q;
		Vd = Vdq.d;
		Vq = Vdq.q;
		Iabc_ref = Transform::to_threephase(Idq_ref, angle_4_control);
		Ia_ref = Iabc_ref.a;
		Ib_ref = Iabc_ref.b;
		angle_ref = angle_4_control;
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