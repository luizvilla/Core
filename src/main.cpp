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
#include "filters.h"
#include <ScopeMimicry.h>
#include "singlePhaseInverter.h"

/*-- Zephyr includes --*/
#include "zephyr/console/console.h"


#define MMC_LEAD 0
#define MMC_SM1 1
#define MMC_SM2 2
#define MMC_SM3 3
#define MMC_SM4 4
#define MMC_SM5 5
#define MMC_SM6 6
#define MMC_SM7 7
#define MMC_SM8 8
#define MMC_SM9 9
#define MMC_SM10 10

#define IDLE 0
#define POWER 1
#define LEAD_ERROR 2
#define OVER_VOLTAGE 3
#define UNDER_VOLTAGE 4
#define OVER_CURRENT 5

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST = MMC_SM10;

/* -------------- BOARD IDENTIFICATION ----------------------- */

constexpr uint32_t UID_MMC_LEAD_BOARD = 0x002B002A;
constexpr uint32_t UID_MMC_SM1_BOARD = 0x00330054;
constexpr uint32_t UID_MMC_SM2_BOARD = 0x0033004B;
constexpr uint32_t UID_MMC_SM3_BOARD = 0x00330049;
constexpr uint32_t UID_MMC_SM4_BOARD = 0x0033004C;
constexpr uint32_t UID_MMC_SM5_BOARD = 0x0031001B;
constexpr uint32_t UID_MMC_SM6_BOARD = 0x003B004D;
constexpr uint32_t UID_MMC_SM7_BOARD = 0x11119999;
constexpr uint32_t UID_MMC_SM8_BOARD = 0x1111AAA0;
constexpr uint32_t UID_MMC_SM9_BOARD = 0x1111BBB1;
constexpr uint32_t UID_MMC_SM10_BOARD = 0x1111CCC2;


static uint32_t read_board_uid()
{
    static volatile uint32_t *const uid0 =
        reinterpret_cast<volatile uint32_t *>(0x1FFF7590UL);
    return *uid0;
}

static uint8_t detect_module_id()
{
    switch (read_board_uid())
    {
    case UID_MMC_LEAD_BOARD:
        return MMC_LEAD;
    case UID_MMC_SM1_BOARD:
        return MMC_SM1;
    case UID_MMC_SM2_BOARD:
        return MMC_SM2;
    case UID_MMC_SM3_BOARD:
        return MMC_SM3;
    case UID_MMC_SM4_BOARD:
        return MMC_SM4;
    case UID_MMC_SM5_BOARD:
        return MMC_SM5;
    case UID_MMC_SM6_BOARD:
        return MMC_SM6;
    case UID_MMC_SM7_BOARD:
        return MMC_SM7;
    case UID_MMC_SM8_BOARD:
        return MMC_SM8;
    case UID_MMC_SM9_BOARD:
        return MMC_SM9;
    case UID_MMC_SM10_BOARD:
        return MMC_SM10;
    default:
        return MMC_SM1;
    }
}

/* -------------- DATA PACKING HELPERS ----------------------- */

constexpr float32_t Cap_voltage_SCALE = 50.0F;
constexpr float32_t Arm_current_SCALE = 50.0F;
constexpr float32_t Arm_current_OFFSET = 25.0F;

static inline uint16_t mmc_encode_voltage(float32_t voltage)
{
    int32_t raw = static_cast<int32_t>((voltage * 4095.0F) / Cap_voltage_SCALE);
    if (raw < 0)
    {
        raw = 0;
    }
    if (raw > 0x0FFF)
    {
        raw = 0x0FFF;
    }
    return static_cast<uint16_t>(raw);
}

/**
 * @brief Decode a raw capacitor voltage value from an MMC frame.
 *
 * @param raw 12-bit encoded capacitor voltage.
 * @return Physical capacitor voltage in volts.
 */
static inline float32_t mmc_decode_voltage(uint16_t raw)
{
    return (Cap_voltage_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F;
}

/**
 * @brief Encode an arm current into the 12-bit transport format.
 *
 * @param current Physical arm current in amperes.
 * @return 12-bit encoded current suitable for MMC frames.
 */
static inline uint16_t mmc_encode_current(float32_t current)
{
    float32_t shifted = current + Arm_current_OFFSET;
    int32_t raw = static_cast<int32_t>((shifted * 4095.0F) / Arm_current_SCALE);
    if (raw < 0)
    {
        raw = 0;
    }
    if (raw > 0x0FFF)
    {
        raw = 0x0FFF;
    }
    return static_cast<uint16_t>(raw);
}

/**
 * @brief Decode a raw arm current value from an MMC frame.
 *
 * @param raw 12-bit encoded arm current.
 * @return Physical arm current in amperes.
 */
static inline float32_t mmc_decode_current(uint16_t raw)
{
    return ((Arm_current_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F) - Arm_current_OFFSET;
}




/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* Auto-detected module ID (uses dummy UIDs for now). */
uint8_t module_ID = detect_module_id(); // The ID of the module, can be set to MMC_LEAD or any other SMx

static uint8_t module_comand; // The command the followers needs to apply
static uint8_t module_command_past;
static bool change_state_command = false; // Flag to change the state of the command
static bool send_idle = false;            // Flag to send idle command from master to followers

constexpr uint8_t MMC_STATUS_CODE_BITS = 3;
constexpr uint32_t MMC_STATUS_CODE_MASK = (1UL << MMC_STATUS_CODE_BITS) - 1U;
constexpr uint32_t MMC_STATUS_UPPER_ARM_MASK = (1UL << MMC_STATUS_CODE_BITS);

/**
 * @brief Frame exchanged over the RS485 communication bus.
 *
 * Structure overview:
 * - `sm_insertion`: bit-packed insertion flags for each submodule.
 * - `capacitor_voltage_raw`: 12-bit encoded capacitor voltage.
 * - `arm_current_raw`: 12-bit encoded arm current.
 * - `status`: 3-bit global status level plus the arm selection flag.
 * - `sm_id`: identifier of the sender (lead or submodule index).
 */
struct MMC_frame
{
    union
    {
        uint16_t raw;
        struct
        {
            uint16_t sm1_inserted : 1;
            uint16_t sm2_inserted : 1;
            uint16_t sm3_inserted : 1;
            uint16_t sm4_inserted : 1;
            uint16_t sm5_inserted : 1;
            uint16_t sm6_inserted : 1;
            uint16_t sm7_inserted : 1;
            uint16_t sm8_inserted : 1;
            uint16_t sm9_inserted : 1;
            uint16_t sm10_inserted : 1;
        } bits;
    } sm_insertion;
    uint16_t capacitor_voltage_raw : 12;
    uint16_t arm_current_raw : 12;
    union
    {
        uint8_t raw;
        struct
        {
            uint8_t status_code : MMC_STATUS_CODE_BITS;
            uint8_t upper_arm_frame : 1;
        } bits;
    } status;
    uint8_t sm_id;
} __packed;

typedef MMC_frame MMC_frame_t;

/**
 * @brief Store an encoded capacitor voltage value inside an MMC frame.
 *
 * @param frame Frame that will carry the voltage information.
 * @param raw 12-bit raw voltage to write into the frame.
 */
static inline void mmc_frame_set_voltage_raw(MMC_frame_t &frame, uint16_t raw)
{
    frame.capacitor_voltage_raw = static_cast<uint16_t>(raw & 0x0FFFU);
}

/**
 * @brief Get the encoded capacitor voltage contained in an MMC frame.
 *
 * @param frame Frame that carries the voltage information.
 * @return 12-bit raw capacitor voltage.
 */
static inline uint16_t mmc_frame_get_voltage_raw(const MMC_frame_t &frame)
{
    return static_cast<uint16_t>(frame.capacitor_voltage_raw & 0x0FFFU);
}

/**
 * @brief Store an encoded arm current value inside an MMC frame.
 *
 * @param frame Frame that will carry the current information.
 * @param raw 12-bit raw current to write into the frame.
 */
static inline void mmc_frame_set_current_raw(MMC_frame_t &frame, uint16_t raw)
{
    frame.arm_current_raw = static_cast<uint16_t>(raw & 0x0FFFU);
}

/**
 * @brief Get the encoded arm current contained in an MMC frame.
 *
 * @param frame Frame that carries the current information.
 * @return 12-bit raw arm current.
 */
static inline uint16_t mmc_frame_get_current_raw(const MMC_frame_t &frame)
{
    return static_cast<uint16_t>(frame.arm_current_raw & 0x0FFFU);
}

/**
 * @brief Set the submodule identifier associated with an MMC frame.
 *
 * @param frame Frame to update.
 * @param id Identifier of the sender (lead or submodule).
 */
static inline void mmc_frame_set_sm_identifier(MMC_frame_t &frame, uint8_t id)
{
    frame.sm_id = id;
}

/**
 * @brief Read the submodule identifier stored inside an MMC frame.
 *
 * @param frame Frame to inspect.
 * @return Sender identifier extracted from the frame.
 */
static inline uint8_t mmc_frame_get_sm_identifier(const MMC_frame_t &frame)
{
    return frame.sm_id;
}

/**
 * @brief Update the insertion flag for a given submodule in an MMC frame.
 *
 * @param frame Frame to modify.
 * @param sm_index Submodule identifier to update.
 * @param inserted Set to true if the submodule is inserted.
 */
static inline void mmc_frame_set_sm_inserted(MMC_frame_t &frame, uint8_t sm_index, bool inserted)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return;
    }
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    if (inserted)
    {
        frame.sm_insertion.raw |= mask;
    }
    else
    {
        frame.sm_insertion.raw &= static_cast<uint16_t>(~mask);
    }
}

/**
 * @brief Check whether a submodule is marked as inserted in an MMC frame.
 *
 * @param frame Frame to inspect.
 * @param sm_index Submodule identifier to check.
 * @return True when the insertion flag is set, false otherwise.
 */
static inline bool mmc_frame_get_sm_inserted(const MMC_frame_t &frame, uint8_t sm_index)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return false;
    }
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    return (frame.sm_insertion.raw & mask) != 0U;
}

/**
 * @brief Set the global status level encoded inside an MMC frame.
 *
 * @param frame Frame to modify.
 * @param status_code 3-bit status value (IDLE, POWER, error levels).
 */
static inline void mmc_frame_set_status_code(MMC_frame_t &frame, uint8_t status_code)
{
    frame.status.raw &= ~MMC_STATUS_CODE_MASK;
    frame.status.raw |= static_cast<uint32_t>(status_code & MMC_STATUS_CODE_MASK);
}

/**
 * @brief Retrieve the global status level encoded inside an MMC frame.
 *
 * @param frame Frame to inspect.
 * @return 3-bit status value (IDLE, POWER, error levels).
 */
static inline uint8_t mmc_frame_get_status_code(const MMC_frame_t &frame)
{
    return static_cast<uint8_t>(frame.status.raw & MMC_STATUS_CODE_MASK);
}

/**
 * @brief Mark whether the frame data describes the upper arm.
 *
 * @param frame Frame to update.
 * @param is_upper_arm True when the frame belongs to the upper arm.
 */
static inline void mmc_frame_set_upper_arm_flag(MMC_frame_t &frame, bool is_upper_arm)
{
    if (is_upper_arm)
    {
        frame.status.raw |= MMC_STATUS_UPPER_ARM_MASK;
    }
    else
    {
        frame.status.raw &= ~MMC_STATUS_UPPER_ARM_MASK;
    }
}

/**
 * @brief Determine whether the MMC frame is associated with the upper arm.
 *
 * @param frame Frame to inspect.
 * @return True when the upper arm flag is set, false otherwise.
 */
static inline bool mmc_frame_is_upper_arm(const MMC_frame_t &frame)
{
    return (frame.status.raw & MMC_STATUS_UPPER_ARM_MASK) != 0U;
}

/**
 * @brief Determine if a module identifier corresponds to the upper arm.
 *
 * @param id Module identifier under test.
 * @return True when the module belongs to the upper arm side.
 */
static inline bool mmc_is_upper_arm_module(uint8_t id)
{
    if (id == MMC_LEAD)
    {
        return true;
    }
    if (id < MMC_SM_FIRST || id > MMC_SM_LAST)
    {
        return false;
    }
    uint8_t offset = static_cast<uint8_t>(id - MMC_SM_FIRST);
    return offset < (MMC_SM_COUNT / 2);
}

static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

float32_t MMC_capacitor_voltage[MMC_SM_COUNT];
float32_t MMC_arm_current[MMC_SM_COUNT];

constexpr size_t MMC_FRAME_SIZE = sizeof(MMC_frame_t);

uint8_t buffer_tx[MMC_FRAME_SIZE];
uint8_t buffer_rx[MMC_FRAME_SIZE];

float32_t Cap_voltage = 0.0f;
static float32_t Arm_current = 0.0f;

uint32_t counter_timer = 0;
uint32_t counter_receive = 0;

uint8_t received_serial_char; // Variable to store the received character from the serial interface
int8_t CommTask_num;

static bool master = false;

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

serial_interface_menu_mode mode = IDLEMODE;

void loop_communication_task(); // Code to be executed in the communication task

/* --------------- Firmware CVB variables ------------------*/

/* [us] period of the control task (=critical task) */
static uint32_t control_task_period = 200; // 100 µs
static float32_t Ts = control_task_period*1.0e-6F;

LowPassFirstOrderFilter i_upper_filter(Ts, 400e-6F);
LowPassFirstOrderFilter i_lower_filter(Ts, 400e-6F);
static float32_t i_lower_filter_value;
static float32_t i_upper_filter_value;
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
static clarke_t inverter_vab_output;
static dqo_t mmc_vdq_out;

static const float32_t MMC_VDQ_REF_MAX_D = 30.0F;
static const float32_t MMC_VDQ_REF_MIN_D = -0.1F;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage for measured value (ctrl task) */
static float meas_data;

/* Scope variables */
static bool enable_acq; // Sets trigger moment if true
static const uint16_t NB_DATAS = 1028; // Number of data acquired
static ScopeMimicry scope(NB_DATAS, 12); // Scope configuration with 5 channels
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
static float32_t i_upper_arm= 1.0F; // Upper arm current, to be substituted by measured current when implementing MMC
static float32_t i_lower_arm= -1.0F; // Lower arm current, to be substituted by measured current when implementing MMC

/* Gate logic */
uint8_t g_u[3] = {0,0,0}; // Gate signals to send to the upper modules
uint8_t g_l[3] = {0,0,0}; // Gate signals to send to the lower modules
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;

/* NLM */
static float32_t m = 1;
static float32_t a = 1;
static float32_t angle;
static const float f0 = 250.F;
static const float w0 = 2 * PI * f0;
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


static void update_measurements(void)
{
    float32_t latest = shield.sensors.getLatestValue(V_HIGH);
    if (latest != NO_VALUE)
    {
        V_high = latest;
        Cap_voltage = V_high;
    }

    latest = shield.sensors.getLatestValue(I1_LOW);
    if (latest != NO_VALUE)
    {
        I1_low_value = latest;
        Arm_current = -I1_low_value + 0.2;
    }
}

void reception_function(void)
{
    dataRX_mmc = *(MMC_frame_t *)buffer_rx;
    uint8_t sender_id = mmc_frame_get_sm_identifier(dataRX_mmc);
    uint8_t status_code = mmc_frame_get_status_code(dataRX_mmc);

    if (module_ID == MMC_LEAD)
    {
        if ((sender_id >= MMC_SM_FIRST) && (sender_id <= MMC_SM_LAST))
        {
            const uint8_t index = sender_id - MMC_SM_FIRST;
            MMC_capacitor_voltage[index] =
                mmc_decode_voltage(mmc_frame_get_voltage_raw(dataRX_mmc));
            MMC_arm_current[index] =
                mmc_decode_current(mmc_frame_get_current_raw(dataRX_mmc));

            if ((status_code >= LEAD_ERROR) && (mode != IDLEMODE))
            {
                mode = IDLEMODE;
                send_idle = false;
            }
        }
    }

    else
    {
        if (sender_id == MMC_LEAD)
        {
            /* retrieving command from lead message*/
            module_comand = static_cast<uint8_t>(
                mmc_frame_get_sm_inserted(dataRX_mmc, module_ID));
            /* retrieving status */
            if (status_code == POWER)
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
        if (sender_id == static_cast<uint8_t>(module_ID - 1))
        {
            dataTX_mmc = dataRX_mmc; // Copy the received data to the transmission data
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_voltage_raw(dataTX_mmc,
                                      mmc_encode_voltage(Cap_voltage));
            mmc_frame_set_current_raw(dataTX_mmc,
                                      mmc_encode_current(Arm_current));
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
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

    const uint32_t board_uid = read_board_uid();
    printk("Board UID: 0x%08" PRIX32 "\n", board_uid);
    master = (module_ID == MMC_LEAD);

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
    if (master == true)
    {
        scope.connectChannel(modulation_signal_upper, "m_u");
        scope.connectChannel(modulation_signal_lower, "m_l");
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
        scope.connectChannel(g_u_1, "g_u_1");
        scope.connectChannel(g_u_2, "g_u_2");
        scope.connectChannel(g_u_3, "g_u_3");
        scope.connectChannel(MMC_capacitor_voltage[0], "v_c_1");
        scope.connectChannel(MMC_capacitor_voltage[1], "v_c_2");
        scope.connectChannel(MMC_capacitor_voltage[2], "v_c_3");
        scope.connectChannel(i_lower_filter_value, "I_arm filtred");
        scope.connectChannel(MMC_arm_current[0],"I_arm");
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
    // case 'h':
    //     /*----------SERIAL INTERFACE MENU----------------------- */
    //     printk(" ________________________________________ \n"
    //            "|     ---- MENU buck voltage mode ----   |\n"
    //            "|     press i : idle mode                |\n"
    //            "|     press p : power mode               |\n"
    //            "|     press u : Vdq_ref.d up by 1 V      |\n"
    //            "|     press j : Vdq_ref.d down by 1 V    |\n"
    //            "|     press r : record data              |\n"
    //            "|     press a : toggle enable_acq var    |\n"
    //            "|________________________________________|\n\n");
    //     /*------------------------------------------------------ */
    //     break;
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
    case 'y':
        if (mmc_inverter_mode == FORMING)
        {
            if (mmc_vdq_ref.d < MMC_VDQ_REF_MAX_D)
            {
                mmc_idq_ref.d += 0.1F;
                mmc_inverter.setIdqRef(mmc_idq_ref);
            }
        }
        break;
    case 'h':
        if (mmc_inverter_mode == FORMING)
        {
            if (mmc_vdq_ref.d > MMC_VDQ_REF_MIN_D)
            {
                mmc_idq_ref.d -= 0.1F;
                mmc_inverter.setIdqRef(mmc_idq_ref);
            }
        }
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
    modules_indexes_upper_arm[0] = 1;
    modules_indexes_upper_arm[1] = 0;
    modules_indexes_upper_arm[2] = 2;

    modules_indexes_lower_arm[0] = 2;
    modules_indexes_lower_arm[1] = 0;
    modules_indexes_lower_arm[2] = 1;

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
    update_measurements();

    if (mode == POWERMODE)
    {
        /* The lead sends commands to the followers */
        if (module_ID == MMC_LEAD)
        {
            /* Connection sequence from NLM */

            /* Run inverter control to derive an AC reference from measured grid values */
            test_angle = mmc_inverter.getTheta();
            vgrid_meas = mmc_vdq_ref.d*ot_sin(test_angle); 

            mmc_dc_bus_voltage = 20.0F;

            // mmc_idq_ref.d = mmc_vdq_ref.d/20;
            // mmc_idq_ref.d = mmc_vdq_ref.d/20;

            mmc_inverter.setVdqRef(mmc_vdq_ref);
            // mmc_inverter.setIdqRef(mmc_idq_ref);
            mmc_inverter.setVBus(mmc_dc_bus_voltage);

            inverter_vab_output = mmc_inverter.getVabOutput();
            // igrid_meas = inverter_vab_output.alpha/20; 

            mmc_inverter.inputProcessing(vgrid_meas, igrid_meas);
            (void)mmc_inverter.calculateDuty();
            mmc_vdq_out = mmc_inverter.getVdqOut();

            
            vab_alpha_command = inverter_vab_output.alpha;

            float32_t normalized_vab = 0.0F;
            if (mmc_dc_bus_voltage > 0.0F)
            {
                normalized_vab = vab_alpha_command / (mmc_dc_bus_voltage-5);
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

            number_of_connected_submodules_upper_arm = round(total_number_of_modules_arm*modulation_signal_upper); // recuperate for scope
            number_of_connected_submodules_lower_arm = round(total_number_of_modules_arm*modulation_signal_lower); // recuperate for scope

            memcpy(modules_capacitor_voltages_upper_arm, MMC_capacitor_voltage, 3 * sizeof(float32_t));
            memcpy(modules_capacitor_voltages_lower_arm, &MMC_capacitor_voltage[3], 3 * sizeof(float32_t));

            i_upper_arm = MMC_arm_current[0];
            i_upper_filter_value = i_upper_filter.calculateWithReturn(i_upper_arm); // filtered current value
            i_upper_arm = i_upper_filter_value;



            i_lower_arm = MMC_arm_current[3];
            i_lower_filter_value = i_lower_filter.calculateWithReturn(i_lower_arm); // filtered current value
            i_lower_arm = i_lower_filter_value;

            sorting(); // Executes the CVB algorithm, chosing which modules to connect

            /* Gate assignment with preference from CVB algorithm */
            g_u_1 = (float)g_u[0];  // recuperate for scope acquisition
            g_u_2 = (float)g_u[1];  // recuperate for scope acquisition
            g_u_3 = (float)g_u[2];  // recuperate for scope acquisition

            g_l_1 = (float)g_l[0];  // recuperate for scope acquisition
            g_l_2 = (float)g_l[1];  // recuperate for scope acquisition
            g_l_3 = (float)g_l[2];  // recuperate for scope acquisition

            sw_timer++;

            dataTX_mmc.sm_insertion.raw = 0U;
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM1, g_u[0] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM2, g_u[1] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM3, g_u[2] != 0U);

            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM4, g_l[0] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM5, g_l[1] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM6, g_l[2] != 0U);

            dataTX_mmc.status.raw = 0U;

            mmc_frame_set_status_code(dataTX_mmc, POWER);
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(Cap_voltage));
            mmc_frame_set_current_raw(dataTX_mmc, mmc_encode_current(Arm_current));
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
           
            communication.rs485.startTransmission();
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
        /* Made to send IDLE flag only once */
        if (!send_idle && module_ID == MMC_LEAD)
        {
            dataTX_mmc.sm_insertion.raw = 0U;
            dataTX_mmc.status.raw = 0U;
            mmc_frame_set_status_code(dataTX_mmc, IDLE);
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(Cap_voltage));
            mmc_frame_set_current_raw(dataTX_mmc, mmc_encode_current(Arm_current));
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
            send_idle = true; // Set the flag to send idle command
        }
    }
    /* Scope data acquisition */
    if (scope_timer == scope_period)
    {
        scope.acquire();
        scope_timer = 0;
    }
    counter_timer++;
    scope_timer++;
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
