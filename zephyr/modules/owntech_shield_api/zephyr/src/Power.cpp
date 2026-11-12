/*
 * Copyright (c) 2023-present LAAS-CNRS
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

/*
 * @date   2024
 *
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @author Jean Alinei <jean.alinei@owntech.org>
 * @author Clément Foucher <clement.foucher@laas.fr>
 */

#include "power_init.h"
#include "Power.h"
#include "SpinAPI.h"
#include <errno.h>
#include <math.h>
#include "owntech_fast_code.h"


hrtim_tu_number_t PowerAPI::spinNumberToTu(uint16_t spin_number)
{
    if(spin_number == 12 || spin_number == 14)
    {
        return PWMA;
    }
    else if(spin_number == 15)
    {
        return PWMB;
    }
    else if(spin_number == 2 || spin_number == 4)
    {
        return PWMC;
    }
    else if(spin_number == 5 || spin_number == 6)
    {
        return PWMD;
    }
    else if(spin_number == 10 || spin_number == 11)
    {
        return PWME;
    }
    else if(spin_number == 7 || spin_number == 9)
    {
        return PWMF;
    }
    else
    {
        return PWMA;
    }
}

void PowerAPI::initBurstMode(leg_t leg)
{
    int8_t startIndex = (leg == ALL) ? 0 : leg;
    int8_t endIndex = (leg == ALL) ? dt_leg_count : leg + 1;

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (dt_burst_mode_capable[i])
        {
            spin.pwm.initBurstMode(spinNumberToTu(dt_pwm_pin[i]));
        }
    }
}

OWNTECH_FAST_CODE void PowerAPI::setBurstMode(int bm_cmp, int bm_per)
{
    spin.pwm.setBurstMode(bm_cmp, bm_per);
}

OWNTECH_FAST_CODE void PowerAPI::startBurstMode()
{
    spin.pwm.startBurstMode();
}

OWNTECH_FAST_CODE void PowerAPI::stopBurstMode()
{
    spin.pwm.stopBurstMode();
}

void PowerAPI::deInitBurstMode()
{
    spin.pwm.deInitBurstMode();
}

void PowerAPI::initMode(leg_t leg,
                        hrtim_switch_convention_t leg_convention,
                        hrtim_pwm_mode_t leg_mode)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;
    uint16_t period = 0;

   /*  If ALL is selected, loop through all legs */
    if (leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        /* Configure the shared clock before the first unit starts the master.
         * Repeating this for another leg resets an initialized up/down leg's
         * stored counting frequency to its undoubled switching frequency. */
        if (hrtim_period_Master_get() == 0U)
        {
            spin.pwm.initVariableFrequency(timer_frequency, timer_min_frequency);
        }

        /* Leg with its own frequency (leg-frequency), before initUnit() */
        if (dt_leg_frequency[i] != 0)
        {
            hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[i]);

            if (spin.pwm.initUnitFrequency(leg_tu, dt_leg_frequency[i]) != 0)
            {
                printk("Leg %d: leg-frequency %u Hz is below min-frequency "
                       "%u Hz, the common frequency is used\n",
                       i, dt_leg_frequency[i], timer_min_frequency);
            }
            else if (timer_frequency % dt_leg_frequency[i] != 0)
            {
                /* The critical task runs on the common frequency: with a
                 * non-integer ratio, the PWM and ADC sampling phase seen
                 * by the task changes from one call to the next */
                printk("Leg %d: leg-frequency %u Hz does not divide the "
                       "common frequency %u Hz\n",
                       i, dt_leg_frequency[i], timer_frequency);
            }

            if (dt_phase_shift[i] != 0)
            {
                printk("Leg %d: no phase shift with leg-frequency\n", i);
            }
        }

        /* Set modulation */
        spin.pwm.setModulation(spinNumberToTu(dt_pwm_pin[i]),
                               dt_modulation[i]);

        /* Configure ADC rollover in center aligned mode */
        spin.pwm.setAdcEdgeTrigger(spinNumberToTu(dt_pwm_pin[i]),
                                   dt_edge_trigger[i]);

        /**
         * Configure which External Event will reset the timer for current mode.
         *
         *   COMPARATOR1_INP/PA1 ----------> + - <----------   DAC3 ch1
         *                                    |
         *                                    |
         *                                    v
         *	                                EEV4
         *
         *   COMPARATOR3_INP/PC1 ----------> + - <----------   DAC1 ch1
         *                                    |
         *                                    |
         *                                    v
         *	                                EEV5
         *
         * /!\ WARNING : Current mode is currently only supported for BUCK /!\
         */
        if (leg_mode == CURRENT_MODE)
        {
            if (dt_current_pin[i] == CM_DAC3)
            {
                spin.pwm.setEev(spinNumberToTu(dt_pwm_pin[i]), EEV4);
            }
            else if (dt_current_pin[i] == CM_DAC1)
            {
                spin.pwm.setEev(spinNumberToTu(dt_pwm_pin[i]), EEV5);
            }
        }

        /* Explicitly select voltage mode too when reinitializing a leg. */
        spin.pwm.setMode(spinNumberToTu(dt_pwm_pin[i]), leg_mode);

        /* Choose which output of the timer unit to control with duty cycle */
        spin.pwm.setSwitchConvention(spinNumberToTu(dt_pwm_pin[i]),
                                     leg_convention);

        /* Initialize leg unit */
        spin.pwm.initUnit(spinNumberToTu(dt_pwm_pin[i]));

        /* Configure PWM initial phase shift */
        spin.pwm.setPhaseShift(spinNumberToTu(dt_pwm_pin[i]),
                               dt_phase_shift[i]);

        /* Configure PWM dead time */
        spin.pwm.setDeadTime(spinNumberToTu(dt_pwm_pin[i]),
                             dt_rising_deadtime[i],
                             dt_falling_deadtime[i]);

        /**
         * Configure PWM ADC trigger.
         */
        if (dt_adc[i] != UNKNOWN_ADC)
        {
            spin.pwm.setAdcDecimation(spinNumberToTu(dt_pwm_pin[i]),
                                      dt_adc_decim[i]);

            spin.pwm.setAdcTrigger(spinNumberToTu(dt_pwm_pin[i]),
                                   dt_adc[i]);

            spin.pwm.enableAdcTrigger(spinNumberToTu(dt_pwm_pin[i]));

            spin.data.configureTriggerSource(dt_adc[i], TRIG_PWM);
        }

        /**
         * Choose which DAC controls the leg in current mode
         */
        if (leg_mode == CURRENT_MODE)
        {
            if (dt_current_pin[i] == CM_DAC1)
            {
                spin.dac.currentModeInit(
                    1,
                    tu_channel[spinNumberToTu(dt_pwm_pin[i])]->pwm_conf.pwm_tu
                );

                spin.comp.initialize(3);
            }
            else if (dt_current_pin[i] == CM_DAC3)
            {
                spin.dac.currentModeInit(
                    3,
                    tu_channel[spinNumberToTu(dt_pwm_pin[i])]->pwm_conf.pwm_tu
                );

                spin.comp.initialize(1);
            }
        }

        /**
         * Only relevant for twist and ownverter hardware, to enable optocouplers
         * for MOSFET driver and connection to electrolytic capacitor
         */
        if (dt_pin_driver[i] != 0)
        {
            spin.gpio.configurePin(dt_pin_driver[i], OUTPUT);
        }
        if (dt_pin_capacitor[i] != 0)
        {
            spin.gpio.configurePin(dt_pin_capacitor[i], OUTPUT);
        }
    }

    alignOwnFrequencyLegs();
}

void PowerAPI::alignOwnFrequencyLegs()
{
    /**
     * Legs with their own frequency are not reset by the master. Restart
     * together the counters of all the legs that share a leg-frequency, so
     * that, e.g., the two legs of an H-bridge are in phase. Called after
     * every init: once the last of them is initialized they are aligned.
     */
    hrtim_tu_number_t group[ALL];

    for (int8_t i = 0; i < dt_leg_count; i++)
    {
        if (dt_leg_frequency[i] == 0)
        {
            continue;
        }

        /* Handle each frequency once, from its first leg */
        bool seen = false;
        for (int8_t j = 0; j < i; j++)
        {
            if (dt_leg_frequency[j] == dt_leg_frequency[i])
            {
                seen = true;
            }
        }
        if (seen)
        {
            continue;
        }

        uint8_t count = 0;
        for (int8_t j = i; j < dt_leg_count; j++)
        {
            hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[j]);
            if (dt_leg_frequency[j] == dt_leg_frequency[i] &&
                spin.pwm.hasOwnFrequency(leg_tu))
            {
                group[count++] = leg_tu;
            }
        }

        if (count > 1)
        {
            spin.pwm.resetCounters(group, count);
        }
    }
}


OWNTECH_FAST_CODE void PowerAPI::setDutyCycle(leg_t leg, float32_t duty_value)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    if (leg == ALL)
    {
        startIndex = 0;
        endIndex = dt_leg_count;
    }
    else
    {
        startIndex = leg;
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[i]);
        uint16_t period = tu_channel[leg_tu]->pwm_conf.period;
        uint16_t value = duty_value * period;
        setDutyCycleRaw(static_cast<leg_t>(i), value);
    }
}

OWNTECH_FAST_CODE void PowerAPI::setDutyCycleRaw(leg_t leg, uint16_t duty_value)
{
    hrtim_tu_number_t leg_tu;
    uint16_t duty_cycle_max_raw;
    uint16_t duty_cycle_min_raw;

    int8_t startIndex = 0;
    int8_t endIndex = 0;    

    /*  If ALL is selected, loop through all legs */
    if (leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        startIndex = leg; /* Treat `leg` as the specific leg index */
        endIndex = leg + 1; /* Only iterate for this specific leg */
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        leg_tu = spinNumberToTu(dt_pwm_pin[i]);

        duty_cycle_max_raw = tu_channel[leg_tu]->pwm_conf.duty_max_user;
        duty_cycle_min_raw = tu_channel[leg_tu]->pwm_conf.duty_min_user;

        /* Clamp the duty cycle to be within the range min to max */
        if (duty_value > duty_cycle_max_raw)
        {
            duty_value = duty_cycle_max_raw;
        }
        else if (duty_value < duty_cycle_min_raw)
        {
            duty_value = duty_cycle_min_raw;
        }
        
        /* User limits are applied above; silicon limits and endpoints are
         * handled once in the shared driver, as in the SPIN API. */
        hrtim_duty_cycle_set(leg_tu, duty_value);
    }
}

OWNTECH_FAST_CODE void PowerAPI::start(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /* If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterates for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t leg_index = startIndex; leg_index < endIndex; leg_index++)
    {
        /**
         * Only relevant for twist hardware, to enable optocouplers
         * for mosfet driver
         */
        if(dt_pin_driver[leg_index] != 0) {
            spin.gpio.setPin(dt_pin_driver[leg_index]);
        }

        if (!dt_output1_inactive[leg_index])
        {
            spin.pwm.startSingleOutput(spinNumberToTu(dt_pwm_pin[leg_index]),
                                       TIMING_OUTPUT1);
        }
        if (!dt_output2_inactive[leg_index])
        {
            spin.pwm.startSingleOutput(spinNumberToTu(dt_pwm_pin[leg_index]),
                                       TIMING_OUTPUT2);
        }
    }
}

OWNTECH_FAST_CODE void PowerAPI::stop(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        /* Stop PWM */
        spin.pwm.stopDualOutput(spinNumberToTu(dt_pwm_pin[i]));

        /**
         * Only relevant for twist hardware, to disable optocouplers for mosfet
         * driver
         */
        if(dt_pin_driver[i] != 0)
        {
            spin.gpio.resetPin(dt_pin_driver[i]);
        }
    }
}

#ifdef CONFIG_SHIELD_TWIST

void PowerAPI::connectCapacitor(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if(dt_pin_capacitor[i] != 0)
        {
            spin.gpio.resetPin(dt_pin_capacitor[i]);
        }
    }
}

void PowerAPI::disconnectCapacitor(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if(dt_pin_capacitor[i] != 0)
        {
            spin.gpio.setPin(dt_pin_capacitor[i]);
        }
    }
}

#endif

#ifndef CONFIG_SHIELD_O2

void PowerAPI::connectDriver(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        endIndex = dt_leg_count; /* retrieves the total number of legs */
    }
    else
    {
        startIndex = leg; /* Treat `leg` as the specific leg index */
        endIndex = leg + 1; /* Only iterate for this specific leg */
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if(dt_pin_driver[i] != 0) {
            spin.gpio.setPin(dt_pin_driver[i]);
        }
    }
}

void PowerAPI::disconnectDriver(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if(dt_pin_driver[i] != 0) {
            spin.gpio.setPin(dt_pin_driver[i]);
        }
    }
}

#endif

void PowerAPI::setSlopeCompensation(leg_t leg,
                                    float32_t set_voltage,
                                    float32_t reset_voltage)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        switch (dt_current_pin[i])
        {
        case CM_DAC1:
            spin.dac.slopeCompensation(1, set_voltage, reset_voltage);
            break;
        case CM_DAC3:
            spin.dac.slopeCompensation(3, set_voltage, reset_voltage);
            break;
        default:
            break;
        }
    }
}

void PowerAPI::setTriggerValue(leg_t leg, float32_t trigger_value)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /* Clamp the trigger value within the acceptable range */
    if (trigger_value > 0.95)
    {
        trigger_value = 0.95;
    }
    else if (trigger_value < 0.05)
    {
        trigger_value = 0.05;
    }

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        spin.pwm.setAdcTriggerInstant(spinNumberToTu(dt_pwm_pin[i]),
                                      trigger_value);
    }
}

void PowerAPI::setPhaseShift(leg_t leg, int16_t phase_shift)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        spin.pwm.setPhaseShift(spinNumberToTu(dt_pwm_pin[i]), phase_shift);
    }
}

void PowerAPI::setDeadTime(leg_t leg,
                           uint16_t ns_rising_dt,
                           uint16_t ns_falling_dt)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        spin.pwm.setDeadTime(spinNumberToTu(dt_pwm_pin[i]),
                             ns_rising_dt, ns_falling_dt);
    }
}

void PowerAPI::setFrequency(uint32_t frequency)
{
    spin.pwm.setFrequency(frequency);

    /* spin.pwm.setFrequency() clamps below timer_min_frequency, so mirror
     * that clamping here to keep timer_frequency consistent with the
     * frequency actually applied to the PWM. */
    if (frequency < timer_min_frequency)
    {
        frequency = timer_min_frequency;
    }

    timer_frequency = frequency;

    for (int8_t i = 0; i < dt_leg_count; i++)
    {
        hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[i]);

        /* Legs with their own frequency did not change */
        if (spin.pwm.hasOwnFrequency(leg_tu))
        {
            continue;
        }

        uint16_t period = tu_channel[leg_tu]->pwm_conf.period;

        tu_channel[leg_tu]->pwm_conf.duty_min_user =
            tu_channel[leg_tu]->pwm_conf.duty_min_user_float * period;
        tu_channel[leg_tu]->pwm_conf.duty_max_user =
            tu_channel[leg_tu]->pwm_conf.duty_max_user_float * period;
    }
}

uint32_t PowerAPI::getFrequency()
{
    return timer_frequency;
}

int8_t PowerAPI::setFrequency(leg_t leg, uint32_t frequency)
{
    if (leg == ALL)
    {
        setFrequency(frequency);
        return 0;
    }

    if (leg < 0 || leg >= dt_leg_count)
    {
        return -1;
    }

    /* Also rescales the duty cycle limits of the leg */
    return spin.pwm.setFrequency(spinNumberToTu(dt_pwm_pin[leg]), frequency);
}

uint32_t PowerAPI::getFrequency(leg_t leg)
{
    if (leg == ALL || leg < 0 || leg >= dt_leg_count)
    {
        return timer_frequency;
    }

    return spin.pwm.getFrequency(spinNumberToTu(dt_pwm_pin[leg]));
}

uint32_t PowerAPI::getFrequencyMin()
{
    return timer_min_frequency;
}


void PowerAPI::setDutyCycleMin(leg_t leg, float32_t duty_cycle){
    int8_t startIndex = 0;
    int8_t endIndex = 0;
    hrtim_tu_number_t leg_tu;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (duty_cycle <= 1.0 && duty_cycle >=0.0){
            leg_tu = spinNumberToTu(dt_pwm_pin[i]);
            uint16_t period = tu_channel[leg_tu]->pwm_conf.period;
            tu_channel[leg_tu]->pwm_conf.duty_min_user = duty_cycle * period;
            tu_channel[leg_tu]->pwm_conf.duty_min_user_float = duty_cycle;
        }
    }
}

void PowerAPI::setDutyCycleMax(leg_t leg, float32_t duty_cycle){
    int8_t startIndex = 0;
    int8_t endIndex = 0;
    hrtim_tu_number_t leg_tu;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (duty_cycle <= 1.0 && duty_cycle >=0.0){
            leg_tu = spinNumberToTu(dt_pwm_pin[i]);
            uint16_t period = tu_channel[leg_tu]->pwm_conf.period;
            tu_channel[leg_tu]->pwm_conf.duty_max_user = duty_cycle * period;
            tu_channel[leg_tu]->pwm_conf.duty_max_user_float = duty_cycle;
        }
    }
}

void PowerAPI::setDutyCycleMinRaw(leg_t leg, uint16_t duty_cycle){
    int8_t startIndex = 0;
    int8_t endIndex = 0;
    hrtim_tu_number_t leg_tu;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (duty_cycle <= 0){
            duty_cycle = 0;
        }
        leg_tu = spinNumberToTu(dt_pwm_pin[i]);
        tu_channel[leg_tu]->pwm_conf.duty_min_user = duty_cycle;
        uint16_t period = tu_channel[leg_tu]->pwm_conf.period;
        tu_channel[leg_tu]->pwm_conf.duty_min_user_float = 
                                            (float32_t)duty_cycle /
                                            (float32_t)period;
        
    }

}

void PowerAPI::setDutyCycleMaxRaw(leg_t leg,uint16_t duty_cycle){

    int8_t startIndex = 0;
    int8_t endIndex = 0;
    hrtim_tu_number_t leg_tu;
    uint16_t period;
    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        leg_tu = spinNumberToTu(dt_pwm_pin[i]);
        period = tu_channel[leg_tu]->pwm_conf.period;
        if (duty_cycle >= period){
            duty_cycle = period; 
        }    
        tu_channel[leg_tu]->pwm_conf.duty_max_user = duty_cycle;
        tu_channel[leg_tu]->pwm_conf.duty_max_user_float = 
                                                (float32_t)duty_cycle /
                                                (float32_t)period;
    }
}

float32_t PowerAPI::getDutyCycleMax(leg_t leg){
    hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[leg]);
    return  tu_channel[leg_tu]->pwm_conf.duty_max_user_float; 
}

uint16_t PowerAPI::getDutyCycleMaxRaw(leg_t leg){
    hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[leg]);
    return  tu_channel[leg_tu]->pwm_conf.duty_max_user; 
}

float32_t PowerAPI::getDutyCycleMin(leg_t leg){
    hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[leg]);
    return  tu_channel[leg_tu]->pwm_conf.duty_min_user_float; 
}

uint16_t PowerAPI::getDutyCycleMinRaw(leg_t leg){
    hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[leg]);
    return  tu_channel[leg_tu]->pwm_conf.duty_min_user; 
}

uint16_t PowerAPI::getPeriod(leg_t leg){
    hrtim_tu_number_t leg_tu = spinNumberToTu(dt_pwm_pin[leg]);
    return  tu_channel[leg_tu]->pwm_conf.period; 
}


void PowerAPI::setAdcDecim(leg_t leg, uint16_t adc_decim)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
         /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        spin.pwm.setAdcDecimation(spinNumberToTu(dt_pwm_pin[i]), adc_decim);
    }
}


void PowerAPI::initBuck(leg_t leg, hrtim_pwm_mode_t leg_mode)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (!dt_pwm_x1_high[i])
        {
            /* PWMx1 is connected in hardware to switch low */
            initMode(static_cast<leg_t>(i), PWMx2, leg_mode);
        }
        else
        {
            /* PWMx1 is connected in hardware to switch high */
            initMode(static_cast<leg_t>(i), PWMx1, leg_mode);
        }
    }
}

void PowerAPI::initBoost(leg_t leg)
{
    int8_t startIndex = 0;
    int8_t endIndex = 0;

    /*  If ALL is selected, loop through all legs */
    if(leg == ALL)
    {
        startIndex = 0;
        /* retrieves the total number of legs */
        endIndex = dt_leg_count;
    }
    else
    {
        /* Treat `leg` as the specific leg index */
        startIndex = leg;
        /* Only iterate for this specific leg */
        endIndex = leg + 1;
    }

    for (int8_t i = startIndex; i < endIndex; i++)
    {
        if (!dt_pwm_x1_high[i])
        {
            /* PWMx1 is connected in hardware to switch low */
            initMode(static_cast<leg_t>(i), PWMx1, VOLTAGE_MODE);
        }
        else
        {
            /* PWMx1 is connected in hardware to switch high */
            initMode(static_cast<leg_t>(i), PWMx2, VOLTAGE_MODE);
        }
    }
}

int PowerAPI::initInverterUnipolar(leg_t leg_a, leg_t leg_b)
{
    return initInverter(leg_a, leg_b, false);
}

int PowerAPI::initInverterBipolar(leg_t leg_a, leg_t leg_b)
{
    return initInverter(leg_a, leg_b, true);
}

int PowerAPI::initInverter(leg_t leg_a, leg_t leg_b, bool bipolar)
{
    /* Reject ALL and invalid leg indices before accessing device-tree arrays. */
    if (static_cast<unsigned>(leg_a) >= dt_leg_count ||
        static_cast<unsigned>(leg_b) >= dt_leg_count)
    {
        return -EINVAL;
    }

    /* An H-bridge needs two different physical legs. */
    if (leg_a == leg_b)
    {
        return -EINVAL;
    }

    hrtim_tu_number_t tu_a = spinNumberToTu(dt_pwm_pin[leg_a]);
    hrtim_tu_number_t tu_b = spinNumberToTu(dt_pwm_pin[leg_b]);

    /* Each leg needs its own compare register, so timer aliases are invalid. */
    if (tu_a == tu_b)
    {
        return -EINVAL;
    }

    uint32_t frequency_a = dt_leg_frequency[leg_a];
    uint32_t frequency_b = dt_leg_frequency[leg_b];

    /* A leg without leg-frequency uses the shield's shared frequency. */
    if (frequency_a == 0)
    {
        frequency_a = timer_frequency;
    }
    if (frequency_b == 0)
    {
        frequency_b = timer_frequency;
    }

    /* Equal switching frequencies are required to keep the carriers aligned. */
    if (frequency_a != frequency_b)
    {
        return -EINVAL;
    }

    /* Zero is not a switching frequency; lower frequencies exceed the
     * resolution selected by the shared HRTIM prescaler. */
    if (frequency_a == 0 || frequency_a < timer_min_frequency)
    {
        return -EINVAL;
    }

    /* Free-running and master-referenced timers cannot share a carrier phase,
     * even when their nominal switching frequencies are equal. */
    bool own_frequency_a = (dt_leg_frequency[leg_a] != 0);
    bool own_frequency_b = (dt_leg_frequency[leg_b] != 0);
    if (own_frequency_a != own_frequency_b)
    {
        return -EINVAL;
    }

    /* Both inverter modes use center-aligned PWM on both legs. */
    if (dt_modulation[leg_a] != UpDwn || dt_modulation[leg_b] != UpDwn)
    {
        return -EINVAL;
    }

    /* The two carriers must have the same zero-phase reference. */
    if (dt_phase_shift[leg_a] != 0 || dt_phase_shift[leg_b] != 0)
    {
        return -EINVAL;
    }

    /* Leg A needs both its high-side and low-side gate outputs. */
    if (dt_output1_inactive[leg_a] || dt_output2_inactive[leg_a])
    {
        return -EINVAL;
    }

    /* Leg B also needs both gate outputs for complementary switching. */
    if (dt_output1_inactive[leg_b] || dt_output2_inactive[leg_b])
    {
        return -EINVAL;
    }

    /* Do not assign a timer to more than one inverter pair. */
    for (unsigned i = 0; i < dt_leg_count; ++i)
    {
        /* Unused pair entries have no timers to reserve. */
        if (!inverter_pairs[i].configured)
        {
            continue;
        }

        hrtim_tu_number_t used_a = spinNumberToTu(dt_pwm_pin[i]);
        hrtim_tu_number_t used_b =
            spinNumberToTu(dt_pwm_pin[inverter_pairs[i].leg_b]);

        /* Compare physical timers too, in case leg names alias the hardware. */
        if (tu_a == used_a || tu_a == used_b ||
            tu_b == used_a || tu_b == used_b)
        {
            return -EBUSY;
        }
    }

    const hrtim_tu_number_t timers[] = {tu_a, tu_b};
    for (hrtim_tu_number_t timer : timers)
    {
        /* Before default initialization, the timer has no valid output map
         * and cannot have been started through the PWM API. */
        if (!hrtim_get_status(timer))
        {
            continue;
        }

        /* Reconfiguring either enabled gate could interrupt active switching.
         * Read the hardware state to include outputs started via SPIN. */
        if (LL_HRTIM_IsEnabledOutput(HRTIM1, tu_channel[timer]->gpio_conf.OUT_H) ||
            LL_HRTIM_IsEnabledOutput(HRTIM1, tu_channel[timer]->gpio_conf.OUT_L))
        {
            return -EBUSY;
        }
    }

    /* Leg A high-side always follows D, using the board's PWM pin mapping. */
    initBuck(leg_a, VOLTAGE_MODE);

    /* Bipolar: leg B low-side follows D, opposite to its high-side.
     * Unipolar: leg B high-side follows its own compare command, 1-D. */
    if (bipolar)
    {
        initBoost(leg_b);
    }
    else
    {
        initBuck(leg_b, VOLTAGE_MODE);
    }

    inverter_pairs[leg_a] = {true, leg_b, bipolar};
    int result = setInverterDutyCycle(leg_a, 0.5f);

    /* Release the pair entry if the initial duty could not be programmed.
     * Outputs remain disabled so the caller can handle the failure. */
    if (result != 0)
    {
        inverter_pairs[leg_a] = {};
        return result;
    }

    /* Latch the initial 50% duty while outputs are disabled. An immediate
     * start() must not expose an old compare value from before initialization. */
    LL_HRTIM_ForceUpdate(HRTIM1, tu_channel[tu_a]->pwm_conf.pwm_tu |
                                tu_channel[tu_b]->pwm_conf.pwm_tu);
    spin.pwm.resetCounters(timers, 2);
    return 0;
}

OWNTECH_FAST_CODE int PowerAPI::setInverterDutyCycle(leg_t leg_a, float32_t duty)
{
    /* Reject ALL and invalid indices before looking up the pair. */
    if (static_cast<unsigned>(leg_a) >= dt_leg_count)
    {
        return -EINVAL;
    }

    /* Only the first leg passed to initInverter identifies a registered pair. */
    if (!inverter_pairs[leg_a].configured)
    {
        return -EINVAL;
    }

    /* NaN and infinity cannot be converted into a hardware compare value. */
    if (!isfinite(duty))
    {
        return -EINVAL;
    }

    const InverterPair &pair = inverter_pairs[leg_a];
    leg_t leg_b = pair.leg_b;
    hrtim_tu_number_t tu_a = spinNumberToTu(dt_pwm_pin[leg_a]);
    hrtim_tu_number_t tu_b = spinNumberToTu(dt_pwm_pin[leg_b]);
    const pwm_conf_t &pwm_a = tu_channel[tu_a]->pwm_conf;
    const pwm_conf_t &pwm_b = tu_channel[tu_b]->pwm_conf;

    /* The same nonzero period is needed for D and its integer complement. */
    if (pwm_a.period == 0 || pwm_a.period != pwm_b.period)
    {
        return -EINVAL;
    }

    /* Equal prescalers make a timer tick the same duration on both legs. */
    if (pwm_a.ckpsc != pwm_b.ckpsc)
    {
        return -EINVAL;
    }

    /* An individual frequency change would break carrier synchronization. */
    if (pwm_a.frequency != pwm_b.frequency ||
        pwm_a.own_frequency != pwm_b.own_frequency)
    {
        return -EINVAL;
    }

    /* Reject a leg that was individually reconfigured to edge-aligned PWM. */
    if (pwm_a.modulation != UpDwn || pwm_b.modulation != UpDwn)
    {
        return -EINVAL;
    }

    /* Current mode can reset a timer independently of its partner. */
    if (pwm_a.pwm_mode != VOLTAGE_MODE || pwm_b.pwm_mode != VOLTAGE_MODE)
    {
        return -EINVAL;
    }

    /* An individual phase shift would invalidate both modulation schemes. */
    if (tu_channel[tu_a]->phase_shift.value != 0 ||
        tu_channel[tu_b]->phase_shift.value != 0)
    {
        return -EINVAL;
    }

    hrtim_switch_convention_t convention_a =
        dt_pwm_x1_high[leg_a] ? PWMx1 : PWMx2;
    hrtim_switch_convention_t convention_b =
        dt_pwm_x1_high[leg_b] ? PWMx1 : PWMx2;

    /* Bipolar PWM drives leg B's low-side instead of its high-side. */
    if (pair.bipolar)
    {
        convention_b = (convention_b == PWMx1) ? PWMx2 : PWMx1;
    }

    /* Refuse updates if either leg's duty-controlled switch was changed. */
    if (tu_channel[tu_a]->switch_conv.convention != convention_a ||
        tu_channel[tu_b]->switch_conv.convention != convention_b)
    {
        return -EINVAL;
    }

    int period = pwm_a.period;

    /* Respect the prescaler's minimum compare value at both pulse edges.
     * At least four ticks also avoids the immediate 100% output-swap path. */
    int margin = MAX(4, (int)HRTIM_MIN_PER_and_CMP_REG_VALUES[pwm_a.ckpsc]);
    int duty_min_raw = MAX(margin, (int)pwm_a.duty_min_user);
    int duty_max_raw = MIN(period - margin, (int)pwm_a.duty_max_user);

    /* Intersect leg A's range with leg B's range, expressed as leg A's D. */
    if (pair.bipolar)
    {
        /* Both compare commands are D. */
        duty_min_raw = MAX(duty_min_raw, (int)pwm_b.duty_min_user);
        duty_max_raw = MIN(duty_max_raw, (int)pwm_b.duty_max_user);
    }
    else
    {
        /* Leg B's compare is 1-D, so its upper limit bounds D from below. */
        duty_min_raw = MAX(duty_min_raw, period - pwm_b.duty_max_user);
        duty_max_raw = MIN(duty_max_raw, period - pwm_b.duty_min_user);
    }

    /* Conflicting leg limits leave no duty that can be applied to both legs. */
    if (duty_min_raw > duty_max_raw)
    {
        return -EINVAL;
    }

    float32_t duty_raw = duty * period;

    /* Clamp before converting to an integer, including commands outside [0,1]. */
    if (duty_raw < duty_min_raw)
    {
        duty_raw = duty_min_raw;
    }
    else if (duty_raw > duty_max_raw)
    {
        duty_raw = duty_max_raw;
    }

    /* Round once and use the exact integer complement to avoid a one-tick bias. */
    uint16_t value_a = static_cast<uint16_t>(duty_raw + 0.5f);
    uint16_t value_b = pair.bipolar ? value_a : period - value_a;

    /* Stage both compares together; they take effect at the same PWM boundary. */
    return hrtim_duty_cycle_pair_set(tu_a, value_a, tu_b, value_b);
}
