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

/**
 * @brief  This example demonstrates how to deploy a Buck converter with
 *         voltage mode control on the Twist power shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"
#include "filters.h"
#include "ScopeMimicry.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;
static bool trace = false;
static bool mppt = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t V2_filt;
static float32_t I2_filt;

static float32_t power_now;
static float32_t power_old;
static float32_t max_power;
static float32_t max_power_old;


static float32_t V_high_max = 60.0F;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.1;
float32_t duty_cycle2 = 0.1;
float32_t duty_cycle_step = 0.01;
float32_t duty_cycle_resolution = 0.001;
float32_t mppt_sign = 1.0F;
float32_t duty_cycle_max = 0.9;
float32_t duty_cycle_min = 0.1;
float32_t VPV_min = 6.0F;
uint8_t count = 0;

static float32_t Ts = control_task_period * 1.0e-6F;

LowPassFirstOrderFilter v2_filter(Ts, 1);
LowPassFirstOrderFilter i2_filter(Ts, 1);

static ScopeMimicry scope(1024, 2); // scope with 1024 points and 2 channels
static bool is_downloading;
static bool trigger = false;

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


/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the power shield in Buck mode
 *  - Initializes the power shield sensors
 *  - Initializes the PID controller
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Buck voltage mode */
    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    shield.sensors.enableDefaultTwistSensors();

	scope.connectChannel(V2_low_value, "V2");
	scope.connectChannel(I2_low_value, "I2");
    scope.set_delay(0.0F);
    scope.set_trigger(a_trigger);
    scope.start();



    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This tasks implements a minimalistic USB serial interface to control
 * the buck converter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ---- MENU buck voltage mode ----   |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press u : duty cycles UP by 1%     |\n"
               "|     press d : duty cycles DOWN by 1%   |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        break;
    case 't':
        trace = true;
        break;
    case 'm':
        mppt = !mppt;		
        break;
    case 'u':
        duty_cycle += duty_cycle_step;
        duty_cycle2 += duty_cycle_step;
        break;
    case 'd':
        duty_cycle -= duty_cycle_step;
        duty_cycle2 -= duty_cycle_step;
        break;
    case 'q':
        duty_cycle_step -= duty_cycle_resolution;
        break;
    case 'w':
        duty_cycle_step += duty_cycle_resolution;
        break;
    case 'a':
        V_high_max -= 1.0F;
        break;
    case 's':
        V_high_max += 1.0F;
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
    case 'e':
        trigger = true;
        scope.start();
    break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * This task mostly logs back measurements to the USB serial interface.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        spin.led.turnOn();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOff();

        if(trace == true && mppt == false){
            spin.led.toggle();
			//resets the maximum power at the beginning of the trace
			if(duty_cycle == duty_cycle_min) max_power_old = 0.0F; 
            
            duty_cycle += duty_cycle_step;
            duty_cycle2 += duty_cycle_step;
			max_power = V2_low_value * -I2_low_value;

			// Finds the maximum power reached during the trace
			if(max_power>max_power_old){
				max_power_old = max_power;
			}
			
            if (V2_low_value<VPV_min){
                 duty_cycle = duty_cycle_min;
                 duty_cycle2 = duty_cycle_min;
                 trace = false;
            }
        }

		if(mppt == true && trace == false){
			count++;
			if (count==3)
			{
            	spin.led.toggle();
				count=0;
			}
			//current convention is negative for power flow from PV to resistor
			power_now = V2_filt * -I2_filt; 

			if(power_now<power_old){
				mppt_sign = -mppt_sign;	
			} 

			duty_cycle += mppt_sign * duty_cycle_step;
			duty_cycle2 += mppt_sign * duty_cycle_step;

			if(duty_cycle>duty_cycle_max) {
				duty_cycle = duty_cycle_max;
				duty_cycle2 = duty_cycle_max;
			}
			if(duty_cycle<duty_cycle_min) {
				duty_cycle = duty_cycle_min;
				duty_cycle2 = duty_cycle_min;
			}

			power_old = power_now;
		}


    }

    if (!is_downloading) {
        /* Prints the data */
        printk("%.3f:", (double)I2_low_value);	/* Prints I2 */
        printk("%.3f:", (double)V2_low_value);  /* Prints V2 */
        printk("%.3f:", (double)duty_cycle);    /* Prints duty cycle */
        printk("%.3f:", (double)duty_cycle2);	/* Prints duty cycle 2 */	
        printk("%.3f:", (double)V_high);        /* Prints V_high */
        printk("%.3f:", (double)power_now);		/* Prints power */
        printk("%.3f:", (double)power_old);		/* Prints old power */
        printk("%.3f:", (double)max_power_old);	/* Prints maximum power reached during the trace */
        printk("%.3f:", (double)mppt);			/* Prints if MPPT is active or not */
        printk("%.3f:", (double)trace);			/* Prints if trace is active or not */
        printk("%.3f:", (double)duty_cycle_step);			/* Prints duty cycle step */
        printk("%.3f:", (double)V_high_max);        /* Prints maximum V_high */
        printk("\n");
    } else {
        dump_scope_datas(scope);
        is_downloading = false;
    }


    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - It runs the PID controller
 *  - It update the PWM signals
 */
void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE ) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE ) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE ) I_high = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    V2_filt = v2_filter.calculateWithReturn(V2_low_value);
    I2_filt = i2_filter.calculateWithReturn(I2_low_value);


    if(V_high>V_high_max) {
        mode = IDLEMODE;
    }

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(LEG1);
            shield.power.stop(LEG2);
        }
        pwm_enable = false;
        duty_cycle = duty_cycle_min;
        duty_cycle2 = duty_cycle_min;
        trace = false;
        mppt = false;
    }
    else if (mode == POWERMODE)
    {
        shield.power.setDutyCycle(LEG1,duty_cycle);
        shield.power.setDutyCycle(LEG2,duty_cycle2);
 
        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(LEG1);
            shield.power.start(LEG2);
        }
    }

    scope.acquire();


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
