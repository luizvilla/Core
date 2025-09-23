/*
 * Copyright (c) 2021-2024 LAAS-CNRS
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
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

//--------------Zephyr----------------------------------------
#include <zephyr/console/console.h>

//--------------OWNTECH APIs----------------------------------
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

//--------------OWNTECH Libraries-----------------------------
#include "pid.h"
// from control library
#include "trigo.h"
#include "filters.h"
// #include "power_ac1phase.h"
#include "ScopeMimicry.h"
#include "control_factory.h"

#define SCOPE_LENGTH 4096
#define NUM_SCOPE_VAR 5

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

static uint32_t control_task_period = 50; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t temp_1_value;
static float32_t temp_2_value;


static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.3;
static uint32_t critical_task_counter;
static uint32_t acquisition_counter;

uint32_t decimation = 1;

static float32_t Udc = 30.0F; // dc voltage supply assumed [V]
static float f0 = 50.0F; // fundamental frequency [Hz]
static float f_init = 50.0F; // initial frequency [Hz]
static float32_t w0 = 2.0F * PI * f0;   // pulsation [rad/s]
/* Sinewave settings */
static float32_t Vgrid_ref; //[V]
static float32_t Vgrid_amplitude_ref = 0.0F; // [V]
static float32_t Vgrid_amplitude = 0.0F; // [V]
static float angle = 0.F; // [rad]
static float theta = 0.F; // [rad]
static float theta_before = 0.F; // [rad]
const float32_t inverse_2pi = 1.5915493667125701904296875E-1;

static float32_t voltage_reference = 15; //voltage reference

/* PID coefficient for a 8.6ms step response*/

static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;


const float f_start     = 50.0f;     // Hz
const float f_end       = 10000.0f;   // Hz
const float T_sweep     = SCOPE_LENGTH*Ts;      // seconds to go from f_start to f_end
const bool  sweep_loop  = true;      // true: loop; false: hold at f_end

// --- Derived ---
const float k_lin  = (f_end - f_start) / T_sweep;   // Hz/s


// the scope help us to record datas during the critical task
// its a library which must be included in platformio.ini
static ScopeMimicry scope(SCOPE_LENGTH, NUM_SCOPE_VAR);
static bool is_downloading;
static bool trigger = false;


//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE,
    CHIRPMODE
};

uint8_t mode = IDLEMODE;



bool a_trigger() {
    return trigger;
}

/**
 * @brief print recorded data of the ScopeMimicry instance to console
 * we use this function in coordination with a miniterm python filter on the host side.
 * `filter_recorded_data.py` to save the data in a file and format them in float.
 *
 * @param scope
 */
void dump_scope_datas(ScopeMimicry &scope)  {
	scope.reset_dump();
    printk("begin record\n");
	while(scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(100);
	}
    printk("end record\n");
}


//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    /* buck voltage mode */
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();
    shield.power.connectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);

    scope.connectChannel(I2_low_value, "I1_low_value");
    scope.connectChannel(V2_low_value, "V1_low_value");
    scope.connectChannel(duty_cycle, "duty_cycle");
	scope.connectChannel(theta, "theta");
	scope.connectChannel(f0, "f0");
    scope.set_delay(0.05F);
    scope.set_trigger(a_trigger);
    scope.start();


    pid.init(pid_params);

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, control_task_period); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        //----------SERIAL INTERFACE MENU-----------------------
        printk(" ________________________________________\n");
        printk("|     ---- MENU buck voltage mode ----   |\n");
        printk("|     press i : idle mode                |\n");
        printk("|     press p : power mode               |\n");
        printk("|     press u : voltage reference UP     |\n");
        printk("|     press d : voltage reference DOWN   |\n");
        printk("|________________________________________|\n\n");
        //------------------------------------------------------
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        break;
    case 'c':
        printk("chirp mode\n");
        f0 = f_start;
        trigger = true;
        mode = CHIRPMODE;
        break;
    case 'u':
        f0 += 50;
        w0 = 2.0F * PI * f0;   // Updates the w0 

        break;
    case 'd':
        f0 -= 50;
        w0 = 2.0F * PI * f0;   // Updates the w0
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
    case 't':
        trigger = true;
		break;    
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
    if (meas_data != NO_VALUE) temp_1_value = meas_data;

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
    if (meas_data != NO_VALUE) temp_2_value = meas_data;



    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
        if (!is_downloading) {
            printk("%.3f:", (double)I1_low_value);
            printk("%.3f:", (double)V1_low_value);
            printk("%.3f:", (double)I_high);
            printk("%.3f:", (double)V_high);
            printk("%.3f:", (double)f0);
            printk("%.3f:", (double)temp_1_value);
            printk("%.3f:", (double)temp_2_value);
            printk("\n");

        } else {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();


        printk("%.3f:", (double)I1_low_value);
        printk("%.3f:", (double)V1_low_value);
        printk("%.3f:", (double)I_high);
        printk("%.3f:", (double)V_high);
        printk("%.3f:", (double)f0);
        printk("%.3f:", (double)temp_1_value);
        printk("%.3f:", (double)temp_2_value);
        printk("\n");

    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
    critical_task_counter++;

    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;



    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);

        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE || mode == CHIRPMODE)
    {
        trigger = true;
        theta_before = theta;
        theta = (ot_modulo_2pi(theta + w0 * Ts));  

		/*  
			The next line implements the chirp 
			it should be commented for fixed frequency operation
		*/
        if(mode == CHIRPMODE){
            f0 += k_lin * Ts;
            if(f0>f_end){
                mode = IDLEMODE;
            }
            scope.acquire();
        }  

        w0 = 2.0F * PI * f0;

        duty_cycle = (ot_sin(theta)/2.5)+0.5;
        shield.power.setDutyCycle(LEG2,duty_cycle);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
        }
    }

    // if (critical_task_counter%decimation == 0 && mode != CHIRPMODE) 
    // {
    //     scope.acquire();
    // }


}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
