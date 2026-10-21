/*
 * Copyright (c) 2026-present LAAS-CNRS
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
 * @brief  Test of the per-timing-unit HRTIM frequency on a Spin alone.
 *
 *         PWMA follows the common frequency (200 kHz), PWMC has its own
 *         100 kHz, PWMD and PWME their own 50 kHz and start aligned. All
 *         units are center-aligned, each with its own duty cycle so they
 *         can be told apart on the scope. See README.md.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();

/*--------------SUPPORT FUNCTIONS DECLARATION----------------- */
/* Prints frequency, period and duty cycle of each unit */
void print_status();
/* Starts or stops the outputs of all the units */
void set_outputs(bool on);

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [Hz] common frequency (master and PWMA), and the lowest allowed: it
 * sets the HRTIM prescaler shared by all the units */
static constexpr uint32_t COMMON_FREQUENCY = 200000;
static constexpr uint32_t MIN_FREQUENCY = 50000;

/* [Hz] own frequencies */
static constexpr uint32_t PWMC_FREQUENCY = 100000;
static constexpr uint32_t PWMDE_FREQUENCY = 50000;

/* [Hz] step of the frequency keys */
static constexpr uint32_t FREQUENCY_STEP = 10000;

/* [ns] dead time on every unit */
static constexpr uint16_t DEAD_TIME_NS = 200;

struct unit_t
{
    hrtim_tu_number_t pwm;
    const char *name;
    const char *pins;
    uint32_t own_frequency;   /* 0: follows the common frequency */
    float32_t duty;
};

static unit_t units[] = {
    {PWMA, "PWMA", "PA8 / PA9   (Spin 12 / 14)", 0, 0.50F},
    {PWMC, "PWMC", "PB12 / PB13 (Spin 2 / 4)", PWMC_FREQUENCY, 0.25F},
    {PWMD, "PWMD", "PB14 / PB15 (Spin 5 / 6)", PWMDE_FREQUENCY, 0.30F},
    {PWME, "PWME", "PC8 / PC9   (Spin 10 / 11)", PWMDE_FREQUENCY, 0.70F},
};
static constexpr uint8_t UNIT_COUNT = sizeof(units) / sizeof(units[0]);

/* PWMD and PWME share their frequency: restarted together */
static hrtim_tu_number_t aligned_pair[2] = {PWMD, PWME};

static uint32_t common_frequency = COMMON_FREQUENCY;
static bool outputs_on = false;

uint8_t received_serial_char;

/*--------------SETUP FUNCTIONS------------------------------- */

void setup_routine()
{
    /* Common frequency and minimum frequency (prescaler), before any unit */
    spin.pwm.initVariableFrequency(COMMON_FREQUENCY, MIN_FREQUENCY);

    for (uint8_t i = 0; i < UNIT_COUNT; i++)
    {
        unit_t &u = units[i];

        /* Own frequency: between initVariableFrequency() and initUnit() */
        if (u.own_frequency != 0 &&
            spin.pwm.initUnitFrequency(u.pwm, u.own_frequency) != 0)
        {
            printk("%s: %u Hz refused (below %u Hz)\n",
                   u.name, u.own_frequency, MIN_FREQUENCY);
        }

        spin.pwm.setModulation(u.pwm, UpDwn);
        spin.pwm.setSwitchConvention(u.pwm, PWMx1);
        spin.pwm.initUnit(u.pwm);
        spin.pwm.setDeadTime(u.pwm, DEAD_TIME_NS, DEAD_TIME_NS);
        spin.pwm.setDutyCycle(u.pwm, u.duty);
    }

    /* PWMD and PWME were enabled one after the other: align them */
    spin.pwm.resetCounters(aligned_pair, 2);

    set_outputs(true);

    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
}

/*--------------LOOP FUNCTIONS-------------------------------- */

void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        printk(" ____________________________________________ \n"
               "|  --- per-unit HRTIM frequency test ---     |\n"
               "|  press p : print frequencies and periods   |\n"
               "|  press o : outputs on / off                |\n"
               "|  press u / j : common frequency +/- 10 kHz |\n"
               "|                (PWMA only)                 |\n"
               "|  press d / c : PWMD frequency +/- 10 kHz   |\n"
               "|                (PWMD only)                 |\n"
               "|  press r : restart PWMD and PWME together  |\n"
               "|____________________________________________|\n\n");
        break;
    case 'p':
        print_status();
        break;
    case 'o':
        set_outputs(!outputs_on);
        printk("outputs %s\n", outputs_on ? "on" : "off");
        break;
    case 'u':
    case 'j':
    {
        uint32_t f = (received_serial_char == 'u') ?
                     common_frequency + FREQUENCY_STEP :
                     common_frequency - FREQUENCY_STEP;
        if (f < MIN_FREQUENCY)
        {
            printk("common frequency cannot go below %u Hz\n", MIN_FREQUENCY);
            break;
        }
        common_frequency = f;
        spin.pwm.setFrequency(common_frequency);
        printk("common frequency %u Hz\n", common_frequency);
        print_status();
        break;
    }
    case 'd':
    case 'c':
    {
        uint32_t current = spin.pwm.getFrequency(PWMD);
        uint32_t f = (received_serial_char == 'd') ?
                     current + FREQUENCY_STEP : current - FREQUENCY_STEP;
        if (spin.pwm.setFrequency(PWMD, f) != 0)
        {
            printk("PWMD: %u Hz refused\n", f);
            break;
        }
        printk("PWMD frequency %u Hz\n", f);
        print_status();
        break;
    }
    case 'r':
        spin.pwm.resetCounters(aligned_pair, 2);
        printk("PWMD and PWME restarted together\n");
        break;
    default:
        break;
    }
}

void loop_application_task()
{
    if (outputs_on)
    {
        spin.led.turnOn();
    }
    else
    {
        spin.led.turnOff();
    }
    task.suspendBackgroundMs(500);
}

/*--------------SUPPORT FUNCTIONS----------------------------- */

void print_status()
{
    printk("unit  pins                         own   frequency  period  duty\n");
    for (uint8_t i = 0; i < UNIT_COUNT; i++)
    {
        unit_t &u = units[i];
        printk("%s  %-27s  %-4s  %6u Hz  %3u us  %.2f\n",
               u.name, u.pins,
               spin.pwm.hasOwnFrequency(u.pwm) ? "yes" : "no",
               spin.pwm.getFrequency(u.pwm),
               spin.pwm.getPeriodUs(u.pwm),
               (double)u.duty);
    }
}

void set_outputs(bool on)
{
    for (uint8_t i = 0; i < UNIT_COUNT; i++)
    {
        if (on)
        {
            spin.pwm.startDualOutput(units[i].pwm);
        }
        else
        {
            spin.pwm.stopDualOutput(units[i].pwm);
        }
    }
    outputs_on = on;
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
