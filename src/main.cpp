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
 * @brief  Test of the per-leg switching frequency (leg-frequency) on an
 *         Ownverter: LEG1 at the common 200 kHz, LEG2 at 100 kHz, LEG3 at
 *         50 kHz (app.overlay).
 *
 *         The PWM outputs are started with the gate drivers DISABLED: the
 *         signals are checked on the Spin pins, no power stage switching.
 *         See README.md.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
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
/* Prints frequency, period and duty cycle of each leg */
void print_status();
/* Starts or stops the PWM outputs, gate drivers untouched */
void set_outputs(bool on);
/* Changes the frequency of a leg by a step */
void step_leg_frequency(leg_t leg, int32_t step);

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [Hz] step of the frequency keys */
static constexpr uint32_t FREQUENCY_STEP = 10000;

struct leg_info_t
{
    leg_t leg;
    hrtim_tu_number_t pwm;
    const char *name;
    const char *pins;
    float32_t duty;
};

static leg_info_t legs[] = {
    {LEG1, PWMA, "LEG1", "PA8 / PA9   (Spin 12 / 14)", 0.50F},
    {LEG2, PWMC, "LEG2", "PB12 / PB13 (Spin 2 / 4)", 0.25F},
    {LEG3, PWME, "LEG3", "PC8 / PC9   (Spin 10 / 11)", 0.30F},
};
static constexpr uint8_t LEG_COUNT = sizeof(legs) / sizeof(legs[0]);

static bool outputs_on = false;
static bool drivers_on = false;

uint8_t received_serial_char;

/*--------------SETUP FUNCTIONS------------------------------- */

void setup_routine()
{
    /* Buck convention, center-aligned. The frequencies come from
     * app.overlay: default-frequency for LEG1, leg-frequency for LEG2 /
     * LEG3. The driver pins are configured as outputs and stay low */
    shield.power.initBuck(ALL);

    for (uint8_t i = 0; i < LEG_COUNT; i++)
    {
        shield.power.setDutyCycle(legs[i].leg, legs[i].duty);
    }

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
        printk(" ______________________________________________ \n"
               "|  --- per-leg frequency test, Ownverter ---   |\n"
               "|  press p : print frequencies and periods     |\n"
               "|  press o : PWM outputs on / off              |\n"
               "|  press u / j : common frequency +/- 10 kHz   |\n"
               "|                (LEG1 only)                   |\n"
               "|  press d / c : LEG2 frequency +/- 10 kHz     |\n"
               "|  press f / v : LEG3 frequency +/- 10 kHz     |\n"
               "|  press s : phase shift 90 deg on LEG3        |\n"
               "|            (must have no effect)             |\n"
               "|  press e / q : gate drivers on / off         |\n"
               "|            (no DC bus during this test!)     |\n"
               "|______________________________________________|\n\n");
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
        uint32_t f = shield.power.getFrequency();
        f = (received_serial_char == 'u') ? f + FREQUENCY_STEP
                                          : f - FREQUENCY_STEP;
        if (f < shield.power.getFrequencyMin())
        {
            printk("common frequency cannot go below %u Hz\n",
                   shield.power.getFrequencyMin());
            break;
        }
        shield.power.setFrequency(f);
        printk("common frequency %u Hz\n", f);
        print_status();
        break;
    }
    case 'd':
        step_leg_frequency(LEG2, FREQUENCY_STEP);
        break;
    case 'c':
        step_leg_frequency(LEG2, -(int32_t)FREQUENCY_STEP);
        break;
    case 'f':
        step_leg_frequency(LEG3, FREQUENCY_STEP);
        break;
    case 'v':
        step_leg_frequency(LEG3, -(int32_t)FREQUENCY_STEP);
        break;
    case 's':
        /* A leg with its own frequency is not referenced to the others */
        shield.power.setPhaseShift(LEG3, 90);
        printk("phase shift 90 deg asked on LEG3 (must not change it)\n");
        break;
    case 'e':
        shield.power.connectDriver(ALL);
        drivers_on = true;
        printk("gate drivers ON: the power stage switches\n");
        break;
    case 'q':
        shield.power.disconnectDriver(ALL);
        drivers_on = false;
        printk("gate drivers off\n");
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
    printk("leg   pins                         frequency  period  duty  drivers %s\n",
           drivers_on ? "ON" : "off");
    for (uint8_t i = 0; i < LEG_COUNT; i++)
    {
        leg_info_t &l = legs[i];
        printk("%s  %-27s  %6u Hz  %3u us  %.2f\n",
               l.name, l.pins,
               shield.power.getFrequency(l.leg),
               spin.pwm.getPeriodUs(l.pwm),
               (double)l.duty);
    }
}

void set_outputs(bool on)
{
    /* spin.pwm, not shield.power.start(): the gate drivers stay as they are */
    for (uint8_t i = 0; i < LEG_COUNT; i++)
    {
        if (on)
        {
            spin.pwm.startDualOutput(legs[i].pwm);
        }
        else
        {
            spin.pwm.stopDualOutput(legs[i].pwm);
        }
    }
    outputs_on = on;
}

void step_leg_frequency(leg_t leg, int32_t step)
{
    uint32_t f = (uint32_t)((int32_t)shield.power.getFrequency(leg) + step);
    if (shield.power.setFrequency(leg, f) != 0)
    {
        printk("LEG%d: %u Hz refused\n", leg + 1, f);
        return;
    }
    printk("LEG%d frequency %u Hz\n", leg + 1, f);
    print_status();
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
