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
 * @brief  This example shows how to a MMC arm works by blinking the onboard LED of the Spin board of the arm modules.
 *         This research was funded in whole by the French National Research Agency (ANR) under the project CARROTS "ANR-24-CE05-0920-01".
 *
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @author Ana Luiza Haas Bezerra <ana-luiza.haas-bezerra@centralesupelec.fr>
 * @author Zaid Jabbar <zaid.jabbar@grenoble-inp.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Jean Alinei <jean.alinei@owntech.org>
 * @author Noemi Lanciotti <noemi.lanciotti@centralesupelec.fr>
 * @author Loïc Quéval <loic.queval@centralesupelec.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "trigo.h"
#include "pid.h"
#include "pr.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>
#include "singlePhaseInverter.h"

/*-- Zephyr includes --*/
#include "zephyr/console/console.h"

/* Boards roles, LEAD = MMC_LEAD */
#define MMC_LEAD 0
#define MMC_M1 1
#define MMC_M2 2
#define MMC_M3 3
#define MMC_M4 4
#define MMC_M5 5
#define MMC_M6 6

/**
 * @brief This function is considering a byte called 'cmd'
 *        which can turn on or off signals.
 *        The signals are identified by their 'id'.
 *        The value 'val' is used to set the signal:
 *        - if val is true, the signal is set to 1
 *        - if val is false, the signal is set to 0
 */
#define SET_SIGNAL(cmd, id, val)   \
    do                             \
    {                              \
        if (val)                   \
            (cmd) |= (1 << (id));  \
        else                       \
            (cmd) &= ~(1 << (id)); \
    } while (0)

/**
 * @brief This function is to get the turn on/off state of a signal
 *        identified by its 'id' from a byte called 'cmd'.
 */
#define GET_SIGNAL(cmd, id) (((cmd) >> (id)) & 0x01)

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* Define module_ID depending on the ID of the board */
uint8_t module_ID = MMC_LEAD; // The ID of the module, can be set to MMC_LEAD or any other SMx

static uint8_t module_comand; // The command the followers needs to apply
static uint8_t module_command_past;
static bool change_state_command = false; // Flag to change the state of the command
static bool send_idle = false;            // Flag to send idle command from master to followers

/**
 * This is a structure that defines the frame
 * that will be sent and received through the RS485 communication.
 * command is a byte that contains the state of the signals
 * Capacitor_Voltage is the voltage of the capacitor
 * ID is the ID of the module
 */
struct MMC_frame
{
    uint8_t command;
    float32_t Capacitor_Voltage;
    uint8_t status;
    uint8_t ID;
} __packed;

typedef MMC_frame MMC_frame_t;
static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

float32_t MMC_capacitor_voltage[6];

uint8_t buffer_tx[7];
uint8_t buffer_rx[7];

float32_t MMC_voltage = 0.0f;

uint32_t counter_timer = 0;
uint32_t counter_receive = 0;

uint8_t received_serial_char; // Variable to store the received character from the serial interface
int8_t CommTask_num;

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

serial_interface_menu_mode mode = IDLEMODE;

void loop_communication_task(); // Code to be executed in the communication task

/* --------------- Firmware CVB variables ------------------*/

/* [us] period of the control task (=critical task) */
static uint32_t control_task_period = 100; // 100 µs
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t vgrid_meas;
static float32_t igrid_meas;
static float32_t vab_alpha_command;
static float32_t mmc_dc_bus_voltage = 30.0F; // default value until RS485 updates arrive

static float32_t test_angle;

static const float32_t GRID_FREQUENCY_HZ = 50.0F;
static const float32_t GRID_W0 = 2.0F * PI * GRID_FREQUENCY_HZ;
static const float32_t GRID_VPK_DEFAULT = 20.0F;
static singlePhaseInverter mmc_inverter;
static inverter_mode mmc_inverter_mode = FORMING;
static dqo_t mmc_vdq_ref;
static dqo_t mmc_idq_ref;
static const float32_t MMC_VDQ_REF_MAX_D = 30.0F;
static const float32_t MMC_VDQ_REF_MIN_D = -0.1F;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage for measured value (ctrl task) */
static float meas_data;

/* Scope variables */
static bool enable_acq; // Sets trigger moment if true
static const uint16_t NB_DATAS = 1028; // Number of data acquired
static ScopeMimicry scope(NB_DATAS, 12); // Scope configuration with MMC control channels
static bool is_downloading; // Records data if true

/* SM switching variables */

static float32_t number_of_connected_submodules_upper_arm;
static float32_t number_of_connected_submodules_lower_arm;
static uint8_t seq_u[6] = {1, 2, 3, 2, 1, 0}; // Connection sequence for upper arm
static uint8_t seq_l[6] = {2, 1, 0, 1, 2, 3}; // Connection sequence for lower arm
static uint8_t counter_seq = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
// static uint32_t f_sw = 2; // 2 Hz = 0.5 s to transition;
// static uint32_t sw_period = 1/(f_sw*control_task_period)*1000000; // 2 Hz = 0.5 s frequency to transition to next connection sequence value;
static uint32_t sw_period = 1000; // 2 Hz = 0.5 s period to transition to next connection sequence value;
static uint32_t scope_period = 1; // scope acquire data every t = scope_period * critical_task_period (100 µs) s;

/* CVB variables */
static float32_t modules_capacitor_voltages_upper_arm[3] = {3.0,5.0,4.0}; // Upper arm modules capacitor voltages artificially generated, to be substituted by measured current when implementing MMC
static uint8_t modules_indexes_upper_arm[3] = {0,1,2}; // Upper arm modules indexes to be sorted with the capacitor voltage vector
static float32_t modules_capacitor_voltages_lower_arm[3] = {3.0,5.0,4.0}; // Lower arm modules capacitor voltages artificially generated, to be substituted by measured current when implementing MMC
static uint8_t modules_indexes_lower_arm[3] = {0,1,2}; // Lower arm modules indexes to be sorted with the capacitor voltage vector
static uint8_t total_number_of_modules_arm= 3;
static int8_t i_upper_arm= 1; // Upper arm current, to be substituted by measured current when implementing MMC
static int8_t i_lower_arm= -1; // Lower arm current, to be substituted by measured current when implementing MMC

/* Gate logic */
uint8_t g_u[3] = {0,0,0}; // Gate signals to send to the upper modules
uint8_t g_l[3] = {0,0,0}; // Gate signals to send to the lower modules
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;

static float32_t Ts = control_task_period * 1e-6F;
static float32_t modulation_signal_upper;
static float32_t modulation_signal_lower;
/* --------------SETUP FUNCTIONS------------------------------- */

/* Function to control the LEDs in the low level */
void config_led_LL()
{
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnON_LL()
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnOFF_LL()
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

/* Trigger function for scope manager */
bool a_trigger()
{
    return enable_acq;
}

void dump_scope_datas(ScopeMimicry &scope)
{
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data) */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
    printk("begin record\n");
    printk("#");
    for (uint16_t k = 0; k < scope.get_nb_channel(); k++)
    {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k = 0; k < buffer_size; k++)
    {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

static void refresh_mmc_dc_bus_voltage_estimate()
{
    /* Estimate the DC link from the latest capacitor voltages reported by the arms */
    float32_t sum = 0.0F;
    uint8_t count = 0U;
    const uint8_t capacitor_count = sizeof(MMC_capacitor_voltage) / sizeof(MMC_capacitor_voltage[0]);
    for (uint8_t idx = 0U; idx < capacitor_count; idx++)
    {
        float32_t voltage = MMC_capacitor_voltage[idx];
        if (voltage > 0.0F)
        {
            sum += voltage;
            count++;
        }
    }
    if (count > 0U)
    {
        mmc_dc_bus_voltage = sum / (float32_t)count;
    }
}

/* RS-485 reception_function: executed when a message is received */
void reception_function(void)
{
    dataRX_mmc = *(MMC_frame_t *)buffer_rx;

    if (module_ID == MMC_LEAD)
    {
        MMC_capacitor_voltage[dataRX_mmc.ID - 1] = dataRX_mmc.Capacitor_Voltage;
        refresh_mmc_dc_bus_voltage_estimate();
        /* TODO: map additional RS485 payload bytes to vgrid_meas and igrid_meas once available. */
    }

    else
    {
        if (dataRX_mmc.ID == MMC_LEAD)
        {
            /* retrievig command from lead message*/
            module_comand = GET_SIGNAL(dataRX_mmc.command, module_ID);
            /* retrieving status */
            if (dataRX_mmc.status == 1)
            {
                mode = POWERMODE;
            }
            else
            {
                mode = IDLEMODE;
            }
        }

        /* The board following the ID of the one who sent will start sending
            the next message */
        if ((dataRX_mmc.ID == module_ID - 1))
        {
            dataTX_mmc = dataRX_mmc; // Copy the received data to the transmission data
            dataTX_mmc.ID = module_ID;
            dataTX_mmc.Capacitor_Voltage = MMC_voltage; /* TODO :uncomment when we get the voltage */
            if (mode == POWERMODE)
            {
                memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
                communication.rs485.startTransmission();
            }
        }
    }
    counter_receive++;
}

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, power shields
 * and tasks.
 *
 * In this example, we spawn a background task and a critical task
 */
void setup_routine()
{

    config_led_LL(); // Configure the LED pin in Low Level

    shield.power.initBuck(ALL);
    /* Declare task */
    uint32_t background_task_number =
        task.createBackground(loop_background_task);

    /* Uncomment following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    shield.sensors.enableDefaultTwistSensors();

    /* Finally, start tasks */
    task.startBackground(background_task_number);
    /* Uncomment following line if you use the critical task */
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    task.startBackground(CommTask_num);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M); // custom configuration for RS485
                                              /* Configure scope channels, what measurements do you want to acquire? */
    if (module_ID == MMC_LEAD)
    {
        scope.connectChannel(modulation_signal_upper, "m_u");
        scope.connectChannel(modulation_signal_lower, "m_l");
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
        scope.connectChannel(g_u_1, "g_u_1");
        scope.connectChannel(g_u_2, "g_u_2");
        scope.connectChannel(g_u_3, "g_u_3");
        scope.connectChannel(g_l_1, "g_l_1");
        scope.connectChannel(g_l_2, "g_l_2");
        scope.connectChannel(g_l_3, "g_l_3");
        scope.connectChannel(vab_alpha_command, "Vab_cmd");
        scope.connectChannel(mmc_dc_bus_voltage, "Vdc_est");
        scope.connectChannel(vgrid_meas, "vgrid_meas");
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.0F);
        scope.start();

        /* Initialise inverter control used to shape MMC insertion sequence */
        vgrid_meas = 0.0F;
        igrid_meas = 0.0F;
        vab_alpha_command = 0.0F;

        mmc_inverter.init(mmc_inverter_mode, GRID_VPK_DEFAULT, GRID_W0, Ts);
        mmc_inverter.setVBus(mmc_dc_bus_voltage);
        dqo_t zero_dqo = {0.0F, 0.0F, 0.0F};
        mmc_vdq_ref = zero_dqo;
        mmc_inverter.setVdqRef(mmc_vdq_ref);
        mmc_inverter.setIdqRef(zero_dqo);
    }
}

/* --------------LOOP FUNCTIONS-------------------------------- */

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
               "|     press u : Vdq_ref.d up by 1 V      |\n"
               "|     press j : Vdq_ref.d down by 1 V    |\n"
               "|     press r : record data              |\n"
               "|     press a : toggle enable_acq var    |\n"
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
        send_idle = false; // Set the flag to send idle command to false 
        break;
    case 'u':
        if (mmc_inverter_mode == FORMING)
        {
            if (mmc_vdq_ref.d < MMC_VDQ_REF_MAX_D)
            {
                mmc_vdq_ref.d += 1.0F;
                mmc_inverter.setVdqRef(mmc_vdq_ref);
            }
        }
        break;
    case 'j':
        if (mmc_inverter_mode == FORMING)
        {
            if (mmc_vdq_ref.d > MMC_VDQ_REF_MIN_D)
            {
                mmc_vdq_ref.d -= 1.0F;
                mmc_inverter.setVdqRef(mmc_vdq_ref);
            }
        }
        break;
    case 'r':
        is_downloading = true;
        enable_acq = false;

        break;
    case 'a':
        enable_acq = true;
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * It runs perpetually. Here a `suspendBackgroundMs` is used to pause during
 * 1000ms between each LED toggles.
 * Hence we expect the LED to blink each second.
 */
void loop_background_task()
{
    if (module_ID == MMC_LEAD)
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
            if (is_downloading)
            {
                dump_scope_datas(scope);
                is_downloading = false;
            }
        }
        if (mode == POWERMODE)
        {
            spin.led.toggle();
            printk("%1.f:", number_of_connected_submodules_upper_arm);
            printk("%1.f:", number_of_connected_submodules_lower_arm);
            printk("%u:", counter_seq);
            printk("%u:", sw_timer);
            printk("%u:", g_u_1);
            printk("%u:", g_u_2);
            printk("%u:", g_u_3);
            printk("%7.3f:", (double)vab_alpha_command);
            printk("%7.3f:", (double)mmc_dc_bus_voltage);
            printk("%7.3f:", (double)mmc_vdq_ref.d);
            printk("\n");
        }
    }

    task.suspendBackgroundMs(500);
}

/* Capacitor Voltage Balancing (CVB) algorithm implementation */
void sorting()
{
    uint8_t counter_loops_sorting = 0;
    while(counter_loops_sorting < 10){ // Sorts modules indexes according to capacitor voltage
            for(uint8_t counter = 0; counter < total_number_of_modules_arm-1; counter++)
            {
                if(modules_capacitor_voltages_upper_arm[counter] > modules_capacitor_voltages_upper_arm[counter + 1])
                {
                    float32_t temp = modules_capacitor_voltages_upper_arm[counter];
                    modules_capacitor_voltages_upper_arm[counter] = modules_capacitor_voltages_upper_arm[counter + 1];
                    modules_capacitor_voltages_upper_arm[counter + 1] = temp;
                    float32_t temp2 = modules_indexes_upper_arm[counter];
                    modules_indexes_upper_arm[counter] = modules_indexes_upper_arm[counter + 1];
                    modules_indexes_upper_arm[counter + 1] = temp2;
                }

                if(modules_capacitor_voltages_lower_arm[counter] > modules_capacitor_voltages_lower_arm[counter + 1])
                {
                    float32_t temp = modules_capacitor_voltages_lower_arm[counter];
                    modules_capacitor_voltages_lower_arm[counter] = modules_capacitor_voltages_lower_arm[counter + 1];
                    modules_capacitor_voltages_lower_arm[counter + 1] = temp;
                    float32_t temp2 = modules_indexes_lower_arm[counter];
                    modules_indexes_lower_arm[counter] = modules_indexes_lower_arm[counter + 1];
                    modules_indexes_lower_arm[counter + 1] = temp2;
                }
            }

            counter_loops_sorting++;
        }
    g_u[0] = 0;
    g_u[1] = 0;
    g_u[2] = 0;
    g_l[0] = 0;
    g_l[1] = 0;
    g_l[2] = 0;
    
    for(uint8_t counter = 0; counter < total_number_of_modules_arm; counter++) // Choses the modules to connect according to sorted indexes
        {
            if(counter < number_of_connected_submodules_upper_arm)
                {
                    if(i_upper_arm>=0)
                    {
                        uint8_t index_smallest_voltage_capacitor_upper_arm = modules_indexes_upper_arm[counter];
                        g_u[index_smallest_voltage_capacitor_upper_arm] = 1;
                    }
                    else{
                        uint8_t higher_index = total_number_of_modules_arm-1-counter;
                        uint8_t index_highest_voltage_capacitor_upper_arm = modules_indexes_upper_arm[higher_index];
                        g_u[index_highest_voltage_capacitor_upper_arm] = 1;
                    }

                }
            if(counter < number_of_connected_submodules_lower_arm)
                {
                    if(i_lower_arm>=0)
                    {
                        uint8_t index_smallest_voltage_capacitor_lower_arm = modules_indexes_lower_arm[counter];
                        g_l[index_smallest_voltage_capacitor_lower_arm] = 1;
                    }
                    else{
                        uint8_t higher_index = total_number_of_modules_arm-1-counter;
                        uint8_t index_highest_voltage_capacitor_lower_arm = modules_indexes_lower_arm[higher_index];
                        g_l[index_highest_voltage_capacitor_lower_arm] = 1;
                    }
                }
        }

}

/**
 * Uncomment lines in setup_routine() to use critical task.
 *
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software
 * function. You can use it to execute an ultra-fast code with
 * the highest priority which cannot be interrupted by the background tasks.
 *
 * In the critical task, you can implement your control algorithm that will
 * run in Real Time and control your power flow.
 */
void loop_critical_task()
{
    if (mode == POWERMODE)
    {
        /* The lead sends commands to the followers */
        if (module_ID == MMC_LEAD)
        {
            /* Connection sequence from NLM */

            /* Run inverter control to derive an AC reference from measured grid values */
            test_angle = mmc_inverter.getTheta();
            vgrid_meas = mmc_vdq_ref.d * ot_sin(test_angle); 
            igrid_meas = mmc_idq_ref.d * ot_sin(test_angle); 
            mmc_dc_bus_voltage = 30.0F;

            mmc_inverter.setVdqRef(mmc_vdq_ref);
            mmc_inverter.setVBus(mmc_dc_bus_voltage);
            mmc_inverter.inputProcessing(vgrid_meas, igrid_meas);
            (void)mmc_inverter.calculateDuty();

            clarke_t inverter_vab_output = mmc_inverter.getVabOutput();
            vab_alpha_command = inverter_vab_output.alpha;

            float32_t normalized_vab = 0.0F;
            if (mmc_dc_bus_voltage > 0.0F)
            {
                normalized_vab = vab_alpha_command / mmc_dc_bus_voltage;
            }

            if (normalized_vab > 1.0F)
            {
                normalized_vab = 1.0F;
            }
            else if (normalized_vab < -1.0F)
            {
                normalized_vab = -1.0F;
            }

            modulation_signal_upper = 0.5F + 0.5F * normalized_vab;
            modulation_signal_lower = 0.5F - 0.5F * normalized_vab;

            if (modulation_signal_upper > 1.0F)
            {
                modulation_signal_upper = 1.0F;
            }
            else if (modulation_signal_upper < 0.0F)
            {
                modulation_signal_upper = 0.0F;
            }

            if (modulation_signal_lower > 1.0F)
            {
                modulation_signal_lower = 1.0F;
            }
            else if (modulation_signal_lower < 0.0F)
            {
                modulation_signal_lower = 0.0F;
            }

            number_of_connected_submodules_upper_arm = round(total_number_of_modules_arm * modulation_signal_upper); // recuperate for scope
            number_of_connected_submodules_lower_arm = round(total_number_of_modules_arm * modulation_signal_lower); // recuperate for scope

            sorting(); // Executes the CVB algorithm, chosing which modules to connect

            /* Gate assignment with preference from CVB algorithm */
            g_u_1 = (float)g_u[0];  // recuperate for scope acquisition
            g_u_2 = (float)g_u[1];  // recuperate for scope acquisition
            g_u_3 = (float)g_u[2];  // recuperate for scope acquisition

            g_l_1 = (float)g_l[0];  // recuperate for scope acquisition
            g_l_2 = (float)g_l[1];  // recuperate for scope acquisition
            g_l_3 = (float)g_l[2];  // recuperate for scope acquisition

            /* Scope data acquisition */
            if (scope_timer == scope_period)
            {
                scope.acquire();
                scope_timer = 0;
            }
            sw_timer++;
            scope_timer++;

            /* Set gate value to be sent to the modules */
            SET_SIGNAL(dataTX_mmc.command, MMC_M1, g_u[0]);
            SET_SIGNAL(dataTX_mmc.command, MMC_M2, g_u[1]);
            SET_SIGNAL(dataTX_mmc.command, MMC_M3, g_u[2]);

            dataTX_mmc.ID = module_ID;
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            dataTX_mmc.status = 1;
            communication.rs485.startTransmission(); // Starts message transmission to other boards
        }
        else
        {
            /* Verifies if command to be ON or OFF changed */
            if (module_comand != module_command_past)
            {
                change_state_command = true; // Set the flag to change the state
            }

            /* Sets LED ON if gate command is 1 or OFF if gate command is 0 */
            if (module_comand)
            {
                if (change_state_command)
                {
                    Led_turnON_LL();
                    change_state_command = false; // Reset the flag
                }
                shield.power.setDutyCycle(LEG1,1.0);
                if (!pwm_enable)
                {
                    pwm_enable = true;
                    shield.power.start(LEG1);
                }
            }
            else if (module_comand == 2)
            {
                if (change_state_command)
                {
                    Led_turnOFF_LL();
                    change_state_command = false; // Reset the flag
                }
                if (pwm_enable == true)
                {
                    shield.power.stop(ALL);
                }
                pwm_enable = false;
            }
            else
            {
                if (change_state_command)
                {
                    Led_turnOFF_LL();
                    change_state_command = false; // Reset the flag
                }
                shield.power.setDutyCycle(LEG1,0.0);
                if (!pwm_enable)
                {
                    pwm_enable = true;
                    shield.power.start(LEG1);
                }
            }
        }
        module_command_past = module_comand; // Update the past command
    }
    else if (mode == IDLEMODE)
    {
        /* Made to send IDLE flag only once to all modules */
        if (!send_idle)
        {
            dataTX_mmc.ID = module_ID;
            dataTX_mmc.status = 0;
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
            send_idle = true; // Set the flag to send idle command to true, meaning that idle mode is active
        }
    }
    counter_timer++;
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
