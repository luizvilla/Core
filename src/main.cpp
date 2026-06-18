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
/* Counts the transferred charge in A.h */
void update_current_counter(float32_t current_value);
/* Converts the A.h counter into a valid voltage curve index */
void current_counter_to_index(float32_t counter_value);

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;


static float32_t I_test;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;
uint32_t init_soc = 10;


/* Voltage reference */
static float32_t voltage_reference = 15;
static float32_t voltage_values[] = {15, 12, 10, 8};
/* Positive electrode sampled on x in [-0.297, 1.005] with 50 points. */
static float32_t positive_potential[] = {
    3.23527F, 3.36715F, 3.42126F, 3.43816F, 3.44493F, 3.44493F, 3.44493F, 3.44831F, 3.44831F,
    3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F,
    3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F,
    3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.44831F, 3.45169F, 3.45169F,
    3.45169F, 3.45169F, 3.45169F, 3.45169F, 3.45169F, 3.45507F, 3.45507F, 3.45845F, 3.45845F,
    3.46522F, 3.47536F, 3.49903F, 3.63092F, 3.79324F
};
/* Negative electrode sampled on x in [0.000, 1.200] with 50 points. */
static float32_t negative_potential[] = {
    0.56812F, 0.38213F, 0.28068F, 0.21643F, 0.19275F, 0.18599F, 0.17585F, 0.15217F, 0.15217F,
    0.14203F, 0.13188F, 0.12512F, 0.11836F, 0.11159F, 0.10821F, 0.09469F, 0.09469F, 0.09469F,
    0.09469F, 0.09469F, 0.09469F, 0.09469F, 0.09469F, 0.09469F, 0.09469F, 0.09469F, 0.09469F,
    0.09469F, 0.08792F, 0.08116F, 0.06763F, 0.06763F, 0.06763F, 0.06763F, 0.06425F, 0.06425F,
    0.06425F, 0.06425F, 0.06425F, 0.06087F, 0.06087F, 0.06087F, 0.05749F, 0.05072F, 0.03720F,
    0.02367F, 0.01353F, 0.00676F, 0.00338F, 0.00000F
};

static float32_t adimmentional_ah_vector[] = {
    0.00000F, 0.02041F, 0.04082F, 0.06122F, 0.08163F, 0.10204F, 0.12245F, 0.14286F, 0.16327F,
    0.18367F, 0.20408F, 0.22449F, 0.24490F, 0.26531F, 0.28571F, 0.30612F, 0.32653F, 0.34694F,
    0.36735F, 0.38776F, 0.40816F, 0.42857F, 0.44898F, 0.46939F, 0.48980F, 0.51020F, 0.53061F,
    0.55102F, 0.57143F, 0.59184F, 0.61224F, 0.63265F, 0.65306F, 0.67347F, 0.69388F, 0.71429F,
    0.73469F, 0.75510F, 0.77551F, 0.79592F, 0.81633F, 0.83673F, 0.85714F, 0.87755F, 0.89796F,
    0.91837F, 0.93878F, 0.95918F, 0.97959F, 1.00000F
};
static constexpr uint32_t electrode_curve_count = sizeof(positive_potential) / sizeof(positive_potential[0]);
static constexpr uint32_t voltage_values_count = sizeof(voltage_values) / sizeof(voltage_values[0]);
static float32_t current_counter = 0;
/* Temporary normalized-capacity step used by the voltage lookup index helper. */
static constexpr float32_t voltage_curve_step_ah = 1.0F / static_cast<float32_t>(electrode_curve_count - 1U);
static float32_t internal_resistor = 0.0; 

/* PID coefficients for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/* Filter */


const float32_t tau = 1.0F;               // constant time
static LowPassFirstOrderFilter average_current_filter(Ts, tau);

uint32_t index_neg = 0U; 
uint32_t index_pos = 0U;

float32_t init_ah = 0.0F; /* Initial A.h value, used to compute the current counter */
    float32_t scale_coefficient_neg = 1.2F; /*  */
    float32_t scale_coefficient_pos = 1.3F; /*  */
    float32_t nominal_capacity = 1.0F; /*  */
    float32_t pos_offset = 0.3F; /*  */




/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/*--------------SETUP FUNCTIONS------------------------------- */

void update_current_counter(float32_t current_value)
{
    // const float32_t average_current = average_current_filter.calculateWithReturn(current_value);
    const float32_t ah_increment = current_value * Ts / 3600.0F;
    const float32_t max_counter = voltage_curve_step_ah * static_cast<float32_t>(voltage_values_count - 1U);

    current_counter += ah_increment;

    if (current_counter < 0.0F)
    {
        current_counter = 0.0F;
    }
    else if (current_counter > max_counter)
    {
        current_counter = max_counter;
    }
}

void current_counter_to_index(float32_t counter_value)
{



    float32_t ah_now = init_ah + counter_value;

    if (ah_now <= 0.0F)
    {
        ah_now = 0.0F;
    }

    index_neg = static_cast<uint32_t>(ah_now / (scale_coefficient_neg * nominal_capacity));
    index_pos = static_cast<uint32_t>((ah_now+pos_offset) / (scale_coefficient_pos * nominal_capacity));

    if (index_neg >= 49U)
    {
        index_neg = 49U;
    }

    if (index_pos >= 49U)
    {
        index_pos = 49U;
    }

}

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
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();

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
               "|     press u : voltage reference UP     |\n"
               "|     press d : voltage reference DOWN   |\n"
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
        I_test += 1;
        break;
    case 'd':
        I_test -= 1;
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

        shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
        shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);

        meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
        if (meas_data != NO_VALUE) temp_1_value = meas_data;

        meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
        if (meas_data != NO_VALUE) temp_2_value = meas_data;




    }


    
    printk("%.3f:", (double)voltage_reference);
    printk("%.3f:", (double)I_test);
    printk("\n");
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
    else if (mode == POWERMODE)
    {

        update_current_counter(I_test);

        current_counter_to_index(current_counter);

        voltage_reference = (positive_potential[index_pos]  - 
                             negative_potential[index_neg]) + 
                             internal_resistor * I_test;

        duty_cycle = pid.calculateWithReturn(voltage_reference, V1_low_value);
        shield.power.setDutyCycle(ALL,duty_cycle);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
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
