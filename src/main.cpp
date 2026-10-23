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
 * @brief  Micro-verter: boost (LEG1 / LEG2, 200 kHz) and single-phase
 *         inverter (LEG3 / LEG4, 50 kHz) in the same firmware.
 *
 *         BOOST_ENABLE and INVERTER_ENABLE select what runs, so that each
 *         converter can be tested alone before running them together.
 *         Measurements, protections and the serial menu are common.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>
#include <string.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"
#include "trigo.h"
#include "singlePhaseInverter.h"

/*--------------WHAT RUNS------------------------------------- */

/* Boost: LEG2 (open loop duty or DC bus voltage loop) */
static constexpr bool BOOST_ENABLE = true;
/* Inverter: LEG3 / LEG4 (fixed u, open-loop sine, grid forming, following) */
static constexpr bool INVERTER_ENABLE = true;

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

/*--------------SUPPORT FUNCTIONS DECLARATION----------------- */
/* Reads all the measurements, in V and A for the inverter side */
void read_measurements();
/* Common protections, returns true on a fault */
bool check_protections();
/* Stops both converters and the inverter bridge supply */
void stop_all();
/* Boost control, called by the critical task when BOOST_ENABLE */
void boost_control();
/* Inverter control, called by the critical task when INVERTER_ENABLE */
void inverter_control();
/* Switches the isolated +12V_HV supply of the inverter bridge (DIS_HV) */
void inverter_supply(bool on);
/* Leg 4 duty cycle for a leg 3 duty cycle, according to the modulation */
float32_t leg4_duty(float32_t d3);
/* Configures LEG4 and the inverter library for a modulation (stopped) */
void set_modulation(modulation_scheme modulation);
/* Clamps a value inside [min, max] */
float32_t saturate(float32_t value, float32_t min, float32_t max);

/*--------------CONSTANTS------------------------------------- */

/* [us] period of the control task (multiple of the 20 us inverter period) */
static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
/* [s] period of the control task */
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;

/* [Hz], [rad/s] grid frequency */
static constexpr float32_t F0 = 50.0F;
static constexpr float32_t W0 = 2.0F * PI * F0;
/* [V] nominal grid peak voltage (library SOGI / PLL tuning) */
static constexpr float32_t GRID_VPK = 325.0F;

/**
 * Measurements: false = raw ADC counts (gain 1, offset 0 on every sensor)
 * until the measurement chains are calibrated; true = V and A on the
 * inverter side with the nominal gains below. Grid forming / following
 * need true (the inverter library works in V and A).
 */
#ifndef MEASUREMENTS_CALIBRATED_DEFAULT
#define MEASUREMENTS_CALIBRATED_DEFAULT false
#endif
static constexpr bool MEASUREMENTS_CALIBRATED = MEASUREMENTS_CALIBRATED_DEFAULT;

/**
 * Measurement chains, nominal values from the schematic (pages 4 to 6),
 * ADC 12 bits on 2.048 V (2000 counts per volt). To calibrate on the
 * board: offsets first, at zero voltage / current.
 *
 * V_AC (V2_LOW, PC0): 887 / (4 x 332k + 887) divider, AMC1301 (x8.2),
 *   x0.487 around 1 V: 2.666 mV/V.
 * I_AC (I_HIGH, PC2): 30 mOhm shunt, AMC1301 (x8.2), x1.21 around 1 V:
 *   0.2977 V/A, full scale +/- 3.36 A.
 * V_DCBus (V_HIGH, PC3): 20k / (4 x 1.13M + 20k) divider, AMC1311 (x1),
 *   x1: 4.405 mV/V.
 * Used only with MEASUREMENTS_CALIBRATED. The boost side (V1_LOW, I1_LOW,
 * I2_LOW) stays in raw ADC counts in both cases.
 */
static constexpr float32_t V_AC_GAIN = 0.18758F;
static constexpr float32_t V_AC_OFFSET = -375.15F;
static constexpr float32_t I_AC_GAIN = 0.0016798F;
static constexpr float32_t I_AC_OFFSET = -3.3595F;
static constexpr float32_t V_DC_GAIN = 0.11351F;
static constexpr float32_t V_DC_OFFSET = 0.0F;

/* DC bus overvoltage trip. Raw: 3200 counts, the former trip (commented as
 * 400 V, 8 counts/V; the schematic gives 8.81 counts/V, i.e. 363 V).
 * Calibrated: 400 V (a 230 V grid needs at least 325 / U_MAX = 361 V) */
static constexpr float32_t V_DC_MAX = MEASUREMENTS_CALIBRATED ? 400.0F : 3200.0F;
/* AC overcurrent trip. Raw: 4000 counts, at the top of the range, for now.
 * Zero current reads about 2000 counts (1 V offset): only a large positive
 * current trips, a negative one reads towards 0 and does not.
 * Calibrated: +/- 3 A (measurement full scale is 3.36 A) */
static constexpr float32_t I_AC_MAX = MEASUREMENTS_CALIBRATED ? 3.0F : 4000.0F;

/* [-] inverter leg duty cycle limits, and the |u| they allow */
static constexpr float32_t INVERTER_DUTY_MIN = 0.05F;
static constexpr float32_t INVERTER_DUTY_MAX = 0.95F;
static constexpr float32_t U_MAX = 0.9F;
/* Minimum DC bus voltage to start the inverter: 10 V calibrated, 100 counts
 * raw (about 11 V with the schematic gain) */
static constexpr float32_t V_DC_MIN_START = MEASUREMENTS_CALIBRATED ? 10.0F : 100.0F;
/* [V] DC bus voltage used by the inverter library before it is measured */
static constexpr float32_t V_DC_FALLBACK = 20.0F;

/* Inverter bridge supply: DIS_HV (PB9, Spin 21) drives EN of U703
 * (VPS8701B, isolated +12V_HV). Level that turns the supply ON: assumed
 * active-high EN as on the VPS8505, TO CONFIRM (TP703 with PB9 high / low) */
static constexpr uint8_t DIS_HV_PIN = 21;
/* Grid relay K1: Relay net, PC13 (Spin 22), drives Q11 directly (page 2):
 * high = relay closed */
static constexpr uint8_t RELAY_PIN = 22;
static constexpr bool HV_SUPPLY_ON_LEVEL = true;
/* [task periods] wait after turning the supply on, before the PWM (20 ms) */
static constexpr uint32_t HV_SUPPLY_SETTLE_TICKS = 200;

/* Inverter modulation at start-up: unipolar (both legs buck, LEG4 = 1 - d) */
static constexpr modulation_scheme MODULATION_DEFAULT = UNIPOLAR;

/* Inverter library PI parameters. false keeps the library defaults (tuned
 * for the TWIST bench); true applies the values below, to tune for the
 * micro-verter output filter */
static constexpr bool INVERTER_CUSTOM_PI = false;
static constexpr float32_t CURRENT_PI_KP = 0.001F;
static constexpr float32_t CURRENT_PI_TI = 0.0003F;
static constexpr float32_t CURRENT_PI_BOUND = 30.0F;
static constexpr float32_t VOLTAGE_PI_KP = 0.01F;
static constexpr float32_t VOLTAGE_PI_TI = 0.003F;
static constexpr float32_t VOLTAGE_PI_BOUND = 30.0F;

/*--------------USER VARIABLES DECLARATIONS------------------- */

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

/* Fault latched by the protections, cleared with 'i' */
static bool fault = false;
static const char *fault_reason = "";

/*--------------BOOST------------------------------------------ */

enum boost_mode
{
    BOOST_OFF = 0,
    BOOST_OPEN_LOOP,   /* manual duty cycle */
    BOOST_CLOSED_LOOP  /* DC bus voltage loop */
};

static uint8_t boost_state = BOOST_OFF;
static bool boost_pwm_on = false;

float32_t duty_cycle = 0.005;

/* [V] DC bus voltage reference */
static float32_t voltage_reference = 15;

/* PID coefficient for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 0.7F;
static float32_t lower_bound = 0.0F;
static PidParams pid_params(TS, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/*--------------INVERTER--------------------------------------- */

enum inverter_operation
{
    INV_OFF = 0,
    INV_FIXED,      /* constant u: DC output u x V_DC (gate checks) */
    INV_OPEN_LOOP,  /* 50 Hz sine, amplitude in V, no feedback */
    INV_FORMING,    /* library FORMING: voltage and current loops */
    INV_FOLLOWING   /* library FOLLOWING: PLL, then current injection */
};

enum inverter_phase
{
    PHASE_STOPPED = 0,
    PHASE_SUPPLY,   /* bridge supply on, waiting before the PWM */
    PHASE_SYNC,     /* following: library running, waiting for the PLL */
    PHASE_RUNNING   /* PWM on */
};

static uint8_t inverter_state = INV_OFF;
static uint8_t inverter_phase = PHASE_STOPPED;
static uint32_t supply_ticks = 0;
static bool inverter_pwm_on = false;
/* Bridge supply state (DIS_HV), and the supply-only request of key z */
static bool hv_supply_on = false;
static bool supply_only_request = false;
/* Grid relay: requested by key r, applied by the critical task; opened by
 * i and by any fault */
static bool relay_request = false;
static bool relay_closed = false;
static modulation_scheme modulation = MODULATION_DEFAULT;

static singlePhaseInverter inverter;

/* [-] INV_FIXED: normalised output voltage u = v / V_DC */
static float32_t inverter_u = 0.0F;
/* [-] INV_OPEN_LOOP: modulation index, u = m sin(wt) (independent of the
 * measurements) */
static float32_t sine_index = 0.0F;
static float32_t sine_angle = 0.0F;
/* [V] INV_FORMING: d-axis voltage reference (peak) */
static float32_t vd_reference = 0.0F;
/* [A] INV_FOLLOWING: d-axis current reference, power injection */
static float32_t id_reference = 0.0F;
static bool following_power_on = false;

/* [V] peak of |V_AC| over the last 20 ms (start check in following) */
static float32_t v_ac_peak = 0.0F;
static float32_t v_ac_peak_running = 0.0F;
static uint32_t v_ac_peak_ticks = 0;

/* Applied duty cycles */
static float32_t duty_leg3 = 0.5F;
static float32_t duty_leg4 = 0.5F;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Whatever the flags, all the legs are initialized (outputs off), so that
 * every measurement is available: LEG1 triggers ADC_1 (boost side), LEG3
 * triggers ADC_2 (inverter side), see app.overlay.
 */
void setup_routine()
{
    /* Boost legs, 200 kHz. LEG1 is not driven, it triggers ADC_1 */
    shield.power.initBoost(LEG1);
    shield.power.initBoost(LEG2);

    shield.power.setDutyCycleMin(LEG1,0.0);
    shield.power.setDutyCycleMax(LEG1,0.7);
    shield.power.setDutyCycleMin(LEG2,0.0);
    shield.power.setDutyCycleMax(LEG2,0.7);

    /* Inverter legs, 50 kHz (leg-frequency in app.overlay) */
    shield.power.initBuck(LEG3);
    shield.power.setDutyCycleMin(LEG3, INVERTER_DUTY_MIN);
    shield.power.setDutyCycleMax(LEG3, INVERTER_DUTY_MAX);

    inverter.init(FORMING, MODULATION_DEFAULT, V_DC_FALLBACK, GRID_VPK, W0, TS);
    if (INVERTER_CUSTOM_PI)
    {
        inverter.setCurrentPi(CURRENT_PI_KP, CURRENT_PI_TI, CURRENT_PI_BOUND);
        inverter.setVoltagePi(VOLTAGE_PI_KP, VOLTAGE_PI_TI, VOLTAGE_PI_BOUND);
    }
    /* LEG4 convention and duty limits follow the modulation */
    set_modulation(MODULATION_DEFAULT);

    /* Inverter bridge supply off until the inverter starts */
    spin.gpio.configurePin(DIS_HV_PIN, OUTPUT);
    inverter_supply(false);

    /* Grid relay open */
    spin.gpio.configurePin(RELAY_PIN, OUTPUT);
    spin.gpio.resetPin(RELAY_PIN);

    shield.sensors.enableDefaultTwistSensors();

    /* All sensors in raw ADC counts: gain 1, offset 0 */
    shield.sensors.setConversionParametersLinear(V1_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I1_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I2_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(V_HIGH,1.0,0.0);
    shield.sensors.setConversionParametersLinear(V2_LOW,1.0,0.0);
    shield.sensors.setConversionParametersLinear(I_HIGH,1.0,0.0);
    if (MEASUREMENTS_CALIBRATED)
    {
        /* DC bus and inverter side in V and A */
        shield.sensors.setConversionParametersLinear(V_HIGH, V_DC_GAIN, V_DC_OFFSET);
        shield.sensors.setConversionParametersLinear(V2_LOW, V_AC_GAIN, V_AC_OFFSET);
        shield.sensors.setConversionParametersLinear(I_HIGH, I_AC_GAIN, I_AC_OFFSET);
    }

    pid.init(pid_params);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This tasks implements a minimalistic USB serial interface to control
 * the boost and the inverter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();

    /* Keys of a disabled converter are refused */
    const char *boost_keys = "pgoluUjJ";
    const char *inverter_keys = "vsfwxkmebnz";
    bool is_boost_key = strchr(boost_keys, received_serial_char) != NULL;
    bool is_inverter_key = strchr(inverter_keys, received_serial_char) != NULL;

    if ((is_boost_key && !BOOST_ENABLE) || (is_inverter_key && !INVERTER_ENABLE))
    {
        printk("%s disabled (BOOST_ENABLE / INVERTER_ENABLE)\n",
               is_boost_key ? "boost" : "inverter");
        return;
    }
    if (fault && (is_boost_key || is_inverter_key))
    {
        printk("fault: %s, press i to clear\n", fault_reason);
        return;
    }

    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     --- MENU micro-verter ---          |\n"
               "|     press i : idle, all off, clear     |\n"
               "|     --- boost (LEG2) ---               |\n"
               "|     press p : open loop duty cycle     |\n"
               "|     press g : DC bus voltage loop      |\n"
               "|     press o / l : duty cycle +/-       |\n"
               "|     press u / j : V_DC ref +/- 1 V     |\n"
               "|     press U / J : V_DC ref +/- 10 V    |\n"
               "|     --- inverter (LEG3/LEG4) ---       |\n"
               "|     press v : fixed u (gate checks)    |\n"
               "|     press s : open-loop sine, index m  |\n"
               "|     press f : grid forming (calib.)    |\n"
               "|     press w : grid following (calib.)  |\n"
               "|     press e : following power on/off   |\n"
               "|     press x : inverter off             |\n"
               "|     press z : bridge supply only on/off|\n"
               "|               (inverter off, no PWM)   |\n"
               "|     press r : grid relay on/off        |\n"
               "|     press k / m : u, m, V or A ref +/- |\n"
               "|     press n / b : unipolar / bipolar   |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        printk("boost %s, inverter %s, modulation %s, measurements %s\n",
               BOOST_ENABLE ? "enabled" : "disabled",
               INVERTER_ENABLE ? "enabled" : "disabled",
               modulation == UNIPOLAR ? "unipolar" : "bipolar",
               MEASUREMENTS_CALIBRATED ? "V / A" : "raw counts");
        break;
    case 'i':
        printk("idle mode\n");
        boost_state = BOOST_OFF;
        inverter_state = INV_OFF;
        supply_only_request = false;
        if (relay_request)
        {
            relay_request = false;
            printk("relay open\n");
        }
        if (fault)
        {
            printk("fault cleared (%s)\n", fault_reason);
            fault = false;
        }
        break;

    /* Boost */
    case 'p':
        printk("boost open loop\n");
        boost_state = BOOST_OPEN_LOOP;
        break;
    case 'g':
        printk("boost voltage loop\n");
        pid.reset(duty_cycle);
        boost_state = BOOST_CLOSED_LOOP;
        break;
    case 'u':
        voltage_reference += 1.0F;
        break;
    case 'j':
        voltage_reference -= 1.0F;
        break;
    case 'U':
        voltage_reference += 10.0F;
        break;
    case 'J':
        voltage_reference -= 10.0F;
        break;
    case 'o':
        duty_cycle += 0.001;
        break;
    case 'l':
        duty_cycle -= 0.001;
        break;

    /* Inverter */
    case 'v':
        printk("inverter fixed u\n");
        inverter_state = INV_FIXED;
        break;
    case 's':
        printk("inverter open-loop sine\n");
        inverter_state = INV_OPEN_LOOP;
        break;
    case 'f':
        if (!MEASUREMENTS_CALIBRATED)
        {
            printk("grid forming needs calibrated measurements "
                   "(MEASUREMENTS_CALIBRATED)\n");
            break;
        }
        printk("inverter grid forming\n");
        if (inverter_state != INV_FORMING)
        {
            vd_reference = 0.0F;
            inverter.setMode(FORMING);
        }
        inverter_state = INV_FORMING;
        break;
    case 'w':
        if (!MEASUREMENTS_CALIBRATED)
        {
            printk("grid following needs calibrated measurements "
                   "(MEASUREMENTS_CALIBRATED)\n");
            break;
        }
        printk("inverter grid following\n");
        if (inverter_state != INV_FOLLOWING)
        {
            id_reference = 0.0F;
            following_power_on = false;
            inverter.setMode(FOLLOWING);
            inverter.setSyncOff();
        }
        inverter_state = INV_FOLLOWING;
        break;
    case 'e':
        following_power_on = !following_power_on;
        printk("following power %s\n", following_power_on ? "on" : "off");
        break;
    case 'x':
        printk("inverter off\n");
        inverter_state = INV_OFF;
        break;
    case 'r':
        if (fault && !relay_request)
        {
            printk("fault: %s, relay stays open (press i)\n", fault_reason);
            break;
        }
        relay_request = !relay_request;
        printk("relay %s\n", relay_request ? "closed" : "open");
        break;
    case 'z':
        /* Bridge supply alone: no PWM, no DC bus check. Applied by the
         * critical task (inverter_control) */
        if (inverter_state != INV_OFF || inverter_phase != PHASE_STOPPED)
        {
            printk("stop the inverter (x) before switching the supply alone\n");
            break;
        }
        supply_only_request = !supply_only_request;
        printk("bridge supply only: %s\n", supply_only_request ? "on" : "off");
        break;
    case 'k':
    case 'm':
    {
        float32_t sign = (received_serial_char == 'k') ? 1.0F : -1.0F;
        if (inverter_state == INV_FIXED)
            inverter_u = saturate(inverter_u + sign * 0.01F, -U_MAX, U_MAX);
        else if (inverter_state == INV_OPEN_LOOP)
            sine_index = saturate(sine_index + sign * 0.05F, 0.0F, U_MAX);
        else if (inverter_state == INV_FORMING)
            vd_reference = saturate(vd_reference + sign * 5.0F, 0.0F, 400.0F);
        else if (inverter_state == INV_FOLLOWING)
            id_reference = saturate(id_reference + sign * 0.1F, -I_AC_MAX, I_AC_MAX);
        break;
    }
    case 'n':
    case 'b':
        if (inverter_state != INV_OFF || inverter_phase != PHASE_STOPPED)
        {
            printk("stop the inverter (x) before changing the modulation\n");
            break;
        }
        set_modulation(received_serial_char == 'n' ? UNIPOLAR : BIPOLAR);
        printk("modulation %s\n", modulation == UNIPOLAR ? "unipolar" : "bipolar");
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
    if (boost_pwm_on || inverter_pwm_on)
    {
        spin.led.turnOn();
    }
    else
    {
        spin.led.turnOff();
    }

    /* Measurements in raw counts unless MEASUREMENTS_CALIBRATED.
       V_low : V_DC : boost duty : V_AC : I_AC : inverter state :
       inverter phase : duty LEG3 : duty LEG4 : sync : bridge supply :
       relay */
    printk("%.3f:", (double)V_low_value);
    printk("%.1f:", (double)V_dc_value);
    printk("%.3f:", (double)duty_cycle);
    printk("%.1f:", (double)V_ac_value);
    printk("%.3f:", (double)I_ac_value);
    printk("%d:", inverter_state);
    printk("%d:", inverter_phase);
    printk("%.3f:", (double)duty_leg3);
    printk("%.3f:", (double)duty_leg4);
    printk("%d:", inverter.getSync());
    printk("%d:", hv_supply_on);
    printk("%d", relay_closed);
    if (fault)
    {
        printk(":FAULT %s", fault_reason);
    }
    printk("\n");
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensor values
 *  - It checks the common protections
 *  - It runs the enabled converters
 */
void loop_critical_task()
{
    read_measurements();

    if (check_protections())
    {
        return;
    }

    /* Grid relay (key r) */
    if (relay_request != relay_closed)
    {
        relay_closed = relay_request;
        if (relay_closed)
        {
            spin.gpio.setPin(RELAY_PIN);
        }
        else
        {
            spin.gpio.resetPin(RELAY_PIN);
        }
    }

    if (BOOST_ENABLE)
    {
        boost_control();
    }

    if (INVERTER_ENABLE)
    {
        inverter_control();
    }
}

/*--------------SUPPORT FUNCTIONS----------------------------- */

void read_measurements()
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

    /* Peak of |V_AC| over one grid period */
    float32_t v_abs = (V_ac_value >= 0.0F) ? V_ac_value : -V_ac_value;
    if (v_abs > v_ac_peak_running)
    {
        v_ac_peak_running = v_abs;
    }
    if (++v_ac_peak_ticks >= (uint32_t)(1.0F / (F0 * TS)))
    {
        v_ac_peak = v_ac_peak_running;
        v_ac_peak_running = 0.0F;
        v_ac_peak_ticks = 0;
    }
}

bool check_protections()
{
    if (!fault)
    {
        if (V_dc_value > V_DC_MAX)
        {
            fault = true;
            fault_reason = "DC bus overvoltage";
        }
        else if (MEASUREMENTS_CALIBRATED ?
                 (I_ac_value > I_AC_MAX || I_ac_value < -I_AC_MAX) :
                 (I_ac_value > I_AC_MAX))
        {
            fault = true;
            fault_reason = "AC overcurrent";
        }

        if (fault)
        {
            printk("FAULT: %s, stopping\n", fault_reason);
        }
    }

    if (fault)
    {
        stop_all();
    }
    return fault;
}

void stop_all()
{
    if (boost_pwm_on || inverter_pwm_on)
    {
        shield.power.stop(ALL);
    }
    boost_pwm_on = false;
    inverter_pwm_on = false;
    boost_state = BOOST_OFF;
    inverter_state = INV_OFF;
    inverter_phase = PHASE_STOPPED;
    supply_only_request = false;
    inverter_supply(false);
    relay_request = false;
    relay_closed = false;
    spin.gpio.resetPin(RELAY_PIN);
}

void boost_control()
{
    if (boost_state == BOOST_OFF)
    {
        if (boost_pwm_on)
        {
            shield.power.stop(LEG2);
            boost_pwm_on = false;
        }
        return;
    }

    if (boost_state == BOOST_CLOSED_LOOP)
    {
        duty_cycle = pid.calculateWithReturn(voltage_reference, V_dc_value);
    }

    shield.power.setDutyCycle(LEG2, duty_cycle);

    if (!boost_pwm_on)
    {
        boost_pwm_on = true;
        shield.power.start(LEG2);
    }
}

void inverter_supply(bool on)
{
    hv_supply_on = on;
    if (on == HV_SUPPLY_ON_LEVEL)
    {
        spin.gpio.setPin(DIS_HV_PIN);
    }
    else
    {
        spin.gpio.resetPin(DIS_HV_PIN);
    }
}

float32_t leg4_duty(float32_t d3)
{
    /* Same rule as singlePhaseInverter::getDutyLeg2() */
    return (modulation == UNIPOLAR) ? 1.0F - d3 : d3;
}

void set_modulation(modulation_scheme new_modulation)
{
    modulation = new_modulation;

    /* Unipolar: LEG4 buck (duty on the high side). Bipolar: LEG4
     * complementary (duty on the low side). The init also re-aligns the
     * LEG3 / LEG4 counters and resets the duty limits to 0.1 - 0.9 */
    if (modulation == UNIPOLAR)
    {
        shield.power.initBuck(LEG4);
    }
    else
    {
        shield.power.initBoost(LEG4);
    }
    shield.power.setDutyCycleMin(LEG4, INVERTER_DUTY_MIN);
    shield.power.setDutyCycleMax(LEG4, INVERTER_DUTY_MAX);

    /* The library keeps its PI parameters through init() */
    inverter.init(FORMING, modulation, V_DC_FALLBACK, GRID_VPK, W0, TS);
}

/**
 * Inverter: bridge supply, start interlock, duty cycles.
 * Sequence: supply on, wait HV_SUPPLY_SETTLE_TICKS, (following: wait for
 * the PLL), PWM on. Stop: PWM off, then supply off.
 */
void inverter_control()
{
    float32_t v_bus = (V_dc_value > V_DC_MIN_START) ? V_dc_value : V_DC_FALLBACK;

    if (inverter_state == INV_OFF)
    {
        if (inverter_pwm_on)
        {
            shield.power.stop(LEG3);
            shield.power.stop(LEG4);
            inverter_pwm_on = false;
        }
        if (inverter_phase != PHASE_STOPPED)
        {
            inverter_supply(false);
            inverter_phase = PHASE_STOPPED;
            supply_only_request = false;
        }
        /* Key z: bridge supply alone, inverter stopped */
        if (supply_only_request != hv_supply_on)
        {
            inverter_supply(supply_only_request);
        }
        sine_angle = 0.0F;
        following_power_on = false;
        duty_leg3 = 0.5F;
        duty_leg4 = 0.5F;
        return;
    }

    /* Bridge supply (already on if key z was used: the start sequence takes
     * it over, x / i / a fault turn it off) */
    if (inverter_phase == PHASE_STOPPED)
    {
        supply_only_request = false;
        inverter_supply(true);
        supply_ticks = 0;
        inverter_phase = PHASE_SUPPLY;
    }

    /* Duty cycles */
    float32_t d3 = 0.5F;
    float32_t d4 = 0.5F;

    switch (inverter_state)
    {
    case INV_FIXED:
        d3 = 0.5F * (1.0F + inverter_u);
        d4 = leg4_duty(d3);
        break;
    case INV_OPEN_LOOP:
    {
        sine_angle = ot_modulo_2pi(sine_angle + W0 * TS);
        float32_t u = sine_index * ot_sin(sine_angle);
        d3 = 0.5F * (1.0F + u);
        d4 = leg4_duty(d3);
        break;
    }
    case INV_FORMING:
    {
        /* The loops only run with the PWM on (no wind-up while waiting) */
        if (inverter_phase != PHASE_RUNNING)
        {
            break;
        }
        dqo_t vdq_ref = {vd_reference, 0.0F, 0.0F};
        inverter.setVBus(v_bus);
        inverter.setVdqRef(vdq_ref);
        d3 = inverter.calculateDuty(V_ac_value, I_ac_value);
        d4 = inverter.getDutyLeg2();
        break;
    }
    case INV_FOLLOWING:
    {
        dqo_t idq_ref = {id_reference, 0.0F, 0.0F};
        bool frequency_ok = inverter.getw() > 0.98F * W0 &&
                            inverter.getw() < 1.02F * W0;
        inverter.setVBus(v_bus);
        inverter.setIdqRef(idq_ref);
        inverter.setPowerOn(following_power_on && inverter.getSync() &&
                            frequency_ok && inverter_pwm_on);
        d3 = inverter.calculateDuty(V_ac_value, I_ac_value);
        d4 = inverter.getDutyLeg2();
        break;
    }
    default:
        break;
    }

    duty_leg3 = saturate(d3, INVERTER_DUTY_MIN, INVERTER_DUTY_MAX);
    duty_leg4 = saturate(d4, INVERTER_DUTY_MIN, INVERTER_DUTY_MAX);

    if (inverter_phase == PHASE_SUPPLY)
    {
        if (++supply_ticks < HV_SUPPLY_SETTLE_TICKS)
        {
            return;
        }
        inverter_phase = (inverter_state == INV_FOLLOWING) ? PHASE_SYNC
                                                           : PHASE_RUNNING;
        if (inverter_phase == PHASE_RUNNING)
        {
            /* Start interlock: the bus must be there */
            if (V_dc_value < V_DC_MIN_START)
            {
                printk("inverter: V_DC %.1f V below %.1f V, not started\n",
                       (double)V_dc_value, (double)V_DC_MIN_START);
                inverter_state = INV_OFF;
                return;
            }
        }
    }

    if (inverter_phase == PHASE_SYNC)
    {
        /* Following: PWM only once the PLL is locked and the bus can
         * reproduce the grid voltage */
        if (!inverter.getSync())
        {
            return;
        }
        if (V_dc_value * U_MAX < v_ac_peak)
        {
            return;
        }
        inverter_phase = PHASE_RUNNING;
    }

    shield.power.setDutyCycle(LEG3, duty_leg3);
    shield.power.setDutyCycle(LEG4, duty_leg4);

    if (!inverter_pwm_on)
    {
        inverter_pwm_on = true;
        shield.power.start(LEG3);
        shield.power.start(LEG4);
    }
}

float32_t saturate(float32_t value, float32_t min, float32_t max)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
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
