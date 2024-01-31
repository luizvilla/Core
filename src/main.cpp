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
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"

#include "zephyr/console/console.h"
#include "opalib_control_pid.h"
#include "LCDLib.hpp"

// #define BUCK_BOARD
#define BOOST_BOARD

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------
enum power_mode
{
    MODE_BOOST_ONLY = 0,
    MODE_BACK2BACK,
    MODE_PAC,
};

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;


static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.5;
float32_t duty_cycle_PID = 0.5;

uint32_t incremental_value_3;
uint32_t incremental_value_4;
uint32_t incremental_value_3_old;
uint32_t incremental_value_4_old;

int32_t diff_incremental_value_3;
int32_t diff_incremental_value_4;

float32_t sign_increment = 0;

#define LEFT_BUTTON PA5
#define RIGHT_BUTTON PC8

bool Is_3_pressed = false;
bool Is_4_pressed = false;
bool Is_4_pressed_old = false;
bool change_mode = false;

LCD lcd(PA13, PB10, PB2, PC5, PA7, PA4);

#ifdef BUCK_BOARD
static float32_t voltage_reference = 10; //voltage reference

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

#endif


#ifdef BOOST_BOARD
static float32_t voltage_reference = 40; //voltage reference

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;
#endif

//--------------SETUP FUNCTIONS-------------------------------


/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
   // Setup the hardware first
    spin.version.setBoardVersion(TWIST_v_1_1_2);

    lcd.begin(16,2);
    lcd.setCursor(0, 0);

    /* voltage mode BUCK and BOOST definition */

    #ifdef BUCK_BOARD
    twist.setVersion(shield_TWIST_V1_2);
    twist.initAllBuck();
    #endif

    #ifdef BOOST_BOARD
    twist.setVersion(shield_TWIST_V1_2);
    twist.initAllBoost();
    spin.timer.startLogTimer3IncrementalEncoder();
    spin.timer.startLogTimer4IncrementalEncoder();
    spin.gpio.configurePin(LEFT_BUTTON, INPUT);
    spin.gpio.configurePin(RIGHT_BUTTON, INPUT);
    #endif

    opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);

    data.enableTwistDefaultChannels();

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------


void loop_communication_task()
{
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press s : serial mode              |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
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
        case 'u':
            duty_cycle += 0.05;
            break;
        case 'd':
            duty_cycle -= 0.05;
            break;
        default:
            break;
        }
    }
}


/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    Is_3_pressed = spin.gpio.readPin(LEFT_BUTTON);
    Is_4_pressed_old = Is_4_pressed;
    Is_4_pressed = spin.gpio.readPin(RIGHT_BUTTON);

    if(Is_4_pressed == false && Is_4_pressed_old == true) change_mode = true;

    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
        lcd.clear();
        lcd.printf("IDLE");
        if(change_mode == true){
            mode = POWERMODE;
            change_mode = false;
        }
    }
    else if (mode == POWERMODE)
    {

        spin.led.turnOn();

        lcd.printf("D=%.3f\nVHigh=%.3f",duty_cycle, V_high);

        if(change_mode == true){
            mode = IDLEMODE;
            change_mode = false;
        }
    }


    incremental_value_3_old = incremental_value_3;
    incremental_value_4_old = incremental_value_4;
    incremental_value_3 = spin.timer.getTimer3IncrementalEncoderValue();
    incremental_value_4 = spin.timer.getTimer4IncrementalEncoderValue();
    diff_incremental_value_3 = incremental_value_3 - incremental_value_3_old;
    diff_incremental_value_4 = incremental_value_4 - incremental_value_4_old;

    if (diff_incremental_value_4>0){
        sign_increment = 1;
    }else if (diff_incremental_value_4<0){
        sign_increment = -1;
    }else{
        sign_increment = 0;
    }


    duty_cycle = duty_cycle + sign_increment*0.01;

    #ifdef BOOST_BOARD
    printk("%i:", Is_3_pressed);
    printk("%i:", Is_4_pressed);
    printk("%d:", diff_incremental_value_3);
    printk("%d:", diff_incremental_value_4);
    printk("%f:", duty_cycle_PID);
    #endif
    printk("%f:", duty_cycle);
    printk("%f:", I1_low_value);
    printk("%f:", V1_low_value);
    printk("%f:", I2_low_value);
    printk("%f:", V2_low_value);
    printk("%f:", I_high);
    printk("%f\n", V_high);
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
    meas_data = data.getLatest(I1_LOW);
    if (meas_data < 10000 && meas_data > NO_VALUE)
        I1_low_value = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE)
        V2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data < 10000 && meas_data > NO_VALUE)
        I2_low_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data < 10000 && meas_data > NO_VALUE)
        I_high = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE)
        V_high = meas_data;


    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            twist.stopAll();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {

        #ifdef BUCK_BOARD
        duty_cycle = opalib_control_interleaved_pid_calculation(voltage_reference, V1_low_value);
        twist.setAllDutyCycle(duty_cycle); // For buck/boost voltage mode
        #endif

        #ifdef BOOST_BOARD
        duty_cycle_PID = opalib_control_interleaved_pid_calculation(voltage_reference, V_high);
        twist.setLegDutyCycle(LEG1, duty_cycle_PID);
        twist.setLegDutyCycle(LEG2, duty_cycle);
        #endif



        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startLeg(LEG1);
            #ifdef BOOST_BOARD
            twist.startLeg(LEG2);
            #endif
        }

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
