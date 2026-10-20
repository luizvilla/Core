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
 * @brief  This example shows how to implement a closed loop voltage mode
 *         boost converter with a Twist power shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>
#include <math.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

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
/* Open-loop inverter, called by the critical task */
void inverter_control();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */
static float32_t V_low_value;
static float32_t V_ac_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_ac_value;
static float32_t V_dc_value;

/* Temporary storage for measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.005;

/* Voltage reference */
static float32_t voltage_reference = 15;

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1.e-6F;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* Inverter (LEG3 / LEG4, 50 kHz), open loop, independent of the boost */
enum inverter_mode
{
    INV_OFF = 0,
    INV_FIXED,  /* constant duty cycle d on LEG3, 1 - d on LEG4 */
    INV_SINE    /* 50 Hz unipolar sine: d = 0.5 + 0.5 m sin(wt) */
};

uint8_t inverter_state = INV_OFF;
static bool inverter_enable = false;

/* [-] duty cycle of LEG3 in INV_FIXED mode */
static float32_t inverter_duty = 0.5F;
/* [-] modulation index in INV_SINE mode */
static float32_t modulation_index = 0.0F;
static const float32_t modulation_index_max = 0.9F;
/* [Hz] output frequency in INV_SINE mode */
static const float32_t grid_frequency = 50.0F;
/* [rad] angle of the sine reference */
static float32_t grid_angle = 0.0F;
static float32_t inverter_duty_applied = 0.5F;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the power shield in Boost mode
 *  - Initializes the power shield sensors
 *  - Initializes the PID controller
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Boost voltage mode, LEG1 initialized to activate ADC1 as well */
    shield.power.initBoost(LEG1);
    shield.power.initBoost(LEG2);

    shield.power.setDutyCycleMin(LEG1,0.0);
    shield.power.setDutyCycleMax(LEG1,0.7);
    shield.power.setDutyCycleMin(LEG2,0.0);
    shield.power.setDutyCycleMax(LEG2,0.7);

    /* Inverter legs, 50 kHz (leg-frequency in app.overlay). Buck
     * convention on both: duty cycle = high-side switch, LEG4 gets 1 - d */
    shield.power.initBuck(LEG3);
    shield.power.initBuck(LEG4);

    shield.power.setDutyCycleMin(LEG3,0.05);
    shield.power.setDutyCycleMax(LEG3,0.95);
    shield.power.setDutyCycleMin(LEG4,0.05);
    shield.power.setDutyCycleMax(LEG4,0.95);

    shield.sensors.enableDefaultTwistSensors();

    shield.sensors.setConversionParametersLinear(V1_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(V2_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(V_HIGH,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I_HIGH,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I1_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I2_LOW,1.0,0.0);


    pid.init(pid_params);

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
 * the boost converter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     --- MENU boost voltage mode ---    |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press u : voltage reference UP     |\n"
               "|     press j : voltage reference DOWN   |\n"
               "|     press o : duty cycle UP            |\n"
               "|     press l : duty cycle DOWN          |\n"
               "|     --- inverter (LEG3/LEG4) ---       |\n"
               "|     press v : fixed duty cycle         |\n"
               "|     press s : 50 Hz sine               |\n"
               "|     press x : inverter off             |\n"
               "|     press k : duty / index UP          |\n"
               "|     press m : duty / index DOWN        |\n"
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
    case 'u':
        voltage_reference += 0.5;
        break;
    case 'j':
        voltage_reference -= 0.5;
        break;
    case 'o':
        duty_cycle += 0.001;
        break;
    case 'l':
        duty_cycle -= 0.001;
        break;
    case 'v':
        printk("inverter fixed duty cycle\n");
        inverter_state = INV_FIXED;
        break;
    case 's':
        printk("inverter 50 Hz sine\n");
        inverter_state = INV_SINE;
        break;
    case 'x':
        printk("inverter off\n");
        inverter_state = INV_OFF;
        break;
    case 'k':
        if (inverter_state == INV_SINE)
        {
            modulation_index += 0.01F;
            if (modulation_index > modulation_index_max)
                modulation_index = modulation_index_max;
        }
        else
        {
            inverter_duty += 0.01F;
        }
        break;
    case 'm':
        if (inverter_state == INV_SINE)
        {
            modulation_index -= 0.01F;
            if (modulation_index < 0.0F)
                modulation_index = 0.0F;
        }
        else
        {
            inverter_duty -= 0.01F;
        }
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
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();

    }

    printk("%.3f:", (double)V_low_value);
    printk("%.3f:", (double)V_dc_value);
    printk("%.3f:", (double)duty_cycle);
    printk("%d:", inverter_state);
    printk("%.3f:", (double)inverter_duty_applied);
    printk("%.3f:", (double)modulation_index);
    printk("\n");
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensor values
 *  - It runs the PID controller
 *  - It update the PWM signals
 */
void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V_ac_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_ac_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_dc_value = meas_data;


    if(V_dc_value > 3200.0)  //800 = 100V  1600= 200V 3200= 400V
    {
        mode = IDLEMODE;
        printk("V_dc_value is too high, stopping the converter\n");
    }


    if (mode == IDLEMODE)
    {
        /* Idle stops everything, the inverter included */
        if (pwm_enable == true || inverter_enable == true)
        {
            shield.power.stop(ALL);
        }
        pwm_enable = false;
        inverter_enable = false;
        inverter_state = INV_OFF;
    }
    else if (mode == POWERMODE)
    {
        // duty_cycle = pid.calculateWithReturn(voltage_reference, V_dc_value);
        shield.power.setDutyCycle(LEG2,duty_cycle);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(LEG2);
        }
    }

    inverter_control();
}

/**
 * Open-loop inverter, called by the critical task. The two legs share the
 * same 50 kHz carrier (aligned at init), LEG4 gets 1 - d: unipolar
 * modulation.
 */
void inverter_control()
{
    if (inverter_state == INV_OFF)
    {
        if (inverter_enable)
        {
            shield.power.stop(LEG3);
            shield.power.stop(LEG4);
            inverter_enable = false;
        }
        grid_angle = 0.0F;
        return;
    }

    if (inverter_state == INV_SINE)
    {
        grid_angle += 2.0F * PI * grid_frequency * Ts;
        if (grid_angle >= 2.0F * PI)
        {
            grid_angle -= 2.0F * PI;
        }
        inverter_duty_applied =
            0.5F + 0.5F * modulation_index * sinf(grid_angle);
    }
    else
    {
        inverter_duty_applied = inverter_duty;
    }

    shield.power.setDutyCycle(LEG3, inverter_duty_applied);
    shield.power.setDutyCycle(LEG4, 1.0F - inverter_duty_applied);

    if (!inverter_enable)
    {
        inverter_enable = true;
        shield.power.start(LEG3);
        shield.power.start(LEG4);
    }
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
