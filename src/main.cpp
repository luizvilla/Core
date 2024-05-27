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

#define BUCK_BOARD
// #define BOOST_BOARD

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
    POWERMODE,
    CLOSEDMODE
};

uint8_t mode = IDLEMODE;

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

static bool leg1_pwm_enable = false;            //[bool] state of the PWM (ctrl task)
static bool leg2_pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;


static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.1;
float32_t duty_cycle_PID = 0.1;

uint32_t incremental_value_3;
uint32_t incremental_value_4;
uint32_t incremental_value_3_old;
uint32_t incremental_value_4_old;

int32_t diff_incremental_value_3;
int32_t diff_incremental_value_4;

float32_t sign_increment = 0;
float32_t sign_increment_3 = 0;

float32_t gain_V1   = 0.044924;
float32_t offset_V1 = -89.828;
float32_t gain_V2   = 0.04553;
float32_t offset_V2 = -91.562;
float32_t gain_VH   = 0.029879;
float32_t offset_VH = -0.092;
float32_t gain_I1   = 0.005552;
float32_t offset_I1 = -11.271;
float32_t gain_I2   = 0.005439;
float32_t offset_I2 = -11.26;
float32_t gain_IH   = 0.004986;
float32_t offset_IH = -9.94;


#define LEFT_BUTTON PB11
#define RIGHT_BUTTON PC8

bool Is_3_pressed = false;
bool Is_4_pressed = false;
bool Is_4_pressed_old = false;
bool Is_3_pressed_old = false;

bool change_mode = false;
bool open_loop_mode = false;
bool closed_loop_mode = false;

static uint8_t mode_counter = 0;
LCD lcd(PA13, PB10, PB2, PC5, PA7, PA4);
// LCD ( RS,   E,    D4,  D5,  D6,  D7);

#ifdef BUCK_BOARD
static float32_t voltage_reference = 10; //voltage reference

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

#endif


#ifdef BOOST_BOARD
static float32_t voltage_reference = 25; //voltage reference

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
    lcd.clear();
    lcd.printf("SYSTEM ON");

    /* voltage mode BUCK and BOOST definition */

    #ifdef BUCK_BOARD
    twist.setVersion(shield_TWIST_V1_2);
    twist.initAllBuck();
    spin.ngnd.turnOn();
    spin.timer.startLogTimer3IncrementalEncoder();
    spin.timer.startLogTimer4IncrementalEncoder();
    spin.gpio.configurePin(LEFT_BUTTON, INPUT);
    spin.gpio.configurePin(RIGHT_BUTTON, INPUT);
    #endif

    #ifdef BOOST_BOARD
    twist.setVersion(shield_TWIST_V1_3);
    twist.initAllBoost();
    spin.timer.startLogTimer3IncrementalEncoder();
    spin.timer.startLogTimer4IncrementalEncoder();
    spin.gpio.configurePin(LEFT_BUTTON, INPUT);
    spin.gpio.configurePin(RIGHT_BUTTON, INPUT);
    #endif



    opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);

    data.enableTwistDefaultChannels();

    data.setParameters(V1_LOW,gain_V1, offset_V1);
    data.setParameters(V2_LOW,gain_V2, offset_V2);
    data.setParameters(V_HIGH,gain_VH, offset_VH);
    data.setParameters(I1_LOW,gain_I1, offset_I1);
    data.setParameters(I2_LOW,gain_I2, offset_I2);
    data.setParameters(I_HIGH,gain_IH, offset_IH);

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
    // Is_3_pressed_old = Is_3_pressed;
    // Is_3_pressed = spin.gpio.readPin(LEFT_BUTTON);

    // if(Is_3_pressed == false && Is_3_pressed_old == true) closed_loop_mode = !closed_loop_mode;

    Is_4_pressed_old = Is_4_pressed;
    Is_4_pressed = spin.gpio.readPin(RIGHT_BUTTON);

    if(Is_4_pressed == false && Is_4_pressed_old == true){
        mode_counter++;
        if (mode_counter == 3) mode_counter = 0;
        // open_loop_mode = !open_loop_mode;
        }


    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
        lcd.clear();
        lcd.printf("IDLE");

        // if(open_loop_mode == true || closed_loop_mode == true){
        //     mode = POWERMODE;
        // }
        if(mode_counter>0) mode = POWERMODE;

    }
    else if (mode == POWERMODE)
    {

        spin.led.turnOn();
        if(mode_counter == 1){
            lcd.printf("CLOSED LOOP MODE\nV=%.2f V2=%.2f",voltage_reference, V2_low_value);
        }else{
            lcd.printf("OPEN LOOP MODE  \nD=%.2f V2=%.2f",duty_cycle, V2_low_value);
            // lcd.printf("DP=%.2f DM=%.2f\nVR=%.2f V1=%.2f",duty_cycle_PID, duty_cycle, voltage_reference, V1_low_value);
        }
        if(mode_counter==0) mode = IDLEMODE;
        // if(open_loop_mode == false && closed_loop_mode == false){
        //     mode = IDLEMODE;
        // }
    }


    incremental_value_3_old = incremental_value_3;
    incremental_value_4_old = incremental_value_4;
    incremental_value_3 = spin.timer.getTimer3IncrementalEncoderValue();
    incremental_value_4 = spin.timer.getTimer4IncrementalEncoderValue();
    diff_incremental_value_3 = incremental_value_3 - incremental_value_3_old;
    diff_incremental_value_4 = incremental_value_4 - incremental_value_4_old;

    //finds out the sign of timer 4
    if (diff_incremental_value_4>0){
        sign_increment = 1;
    }else if (diff_incremental_value_4<0){
        sign_increment = -1;
    }else{
        sign_increment = 0;
    }

    //finds out the sign of timer 3
    if (diff_incremental_value_3>0){
        sign_increment_3 = 1;
    }else if (diff_incremental_value_3<0){
        sign_increment_3 = -1;
    }else{
        sign_increment_3 = 0;
    }

    duty_cycle = duty_cycle + sign_increment*0.01;
    voltage_reference = voltage_reference + sign_increment_3*0.1;

    printk("%i:", Is_3_pressed);
    printk("%i:", Is_3_pressed_old);
    printk("%i:", closed_loop_mode);
    printk("%i:", Is_4_pressed);
    printk("%i:", Is_4_pressed_old);
    printk("%i:", open_loop_mode);
    printk("%d:", diff_incremental_value_3);
    printk("%d:", diff_incremental_value_4);
    printk("%f:", duty_cycle_PID);
    printk("%f:", voltage_reference);
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
        if (leg1_pwm_enable == true){
            twist.stopLeg(LEG1);
            leg1_pwm_enable = false;
        } else if(leg2_pwm_enable == true){
            twist.stopLeg(LEG2);
            leg2_pwm_enable = false;
        }
    }
    else if (mode == POWERMODE)
    {

        #ifdef BUCK_BOARD
        if(mode_counter == 1){ //mode closed loop
            if (!leg2_pwm_enable){
                leg2_pwm_enable = true;
                twist.startLeg(LEG2); //turns on leg1 is the left button is pressed
             }
            duty_cycle_PID = opalib_control_interleaved_pid_calculation(voltage_reference, V2_low_value);
            twist.setLegDutyCycle(LEG2, duty_cycle_PID);
        }else if(mode_counter == 2){
            twist.setLegDutyCycle(LEG2, duty_cycle);
        }else{
            if (leg2_pwm_enable){
                leg2_pwm_enable = false;
                twist.stopLeg(LEG2); // turns off leg 2 if the mode counter is lower than 2
             }
        }
        #endif

        #ifdef BOOST_BOARD
        if(closed_loop_mode == true){
            if (!leg1_pwm_enable){
                leg1_pwm_enable = true;
                twist.startLeg(LEG1); //turns on leg1 is the left button is pressed
             }
            duty_cycle_PID = opalib_control_interleaved_pid_calculation(voltage_reference, V_high);
            twist.setLegDutyCycle(LEG1, duty_cycle_PID);
        }else{
            if (leg1_pwm_enable){
                leg1_pwm_enable = false;
                twist.stopLeg(LEG1); //turns off leg1 is the left button is pressed
             }
        }

        if(open_loop_mode == true){
            if (!leg2_pwm_enable){
                leg2_pwm_enable = true;
                twist.startLeg(LEG2); //turns on leg2 is the right button is pressed
             }
            twist.setLegDutyCycle(LEG2, duty_cycle);
        }else{
            if (leg2_pwm_enable){
                leg2_pwm_enable = false;
                twist.stopLeg(LEG2); // turns off leg 2 if the right button is pressed
             }
        }
        #endif

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
