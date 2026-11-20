/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_data.cpp
 * @brief The ADC-watchdog path goes through data acquisition (ADR 0009, #19):
 *        its channels are enabled through DataAPI, and data acquisition is
 *        started by the Safety API if the first PWM output comes before it.
 */

#include "SpinAPI.h"
#include "safety_trips.h"
#include "adc.h"
#include "ShieldAPI.h"
#include "safety_supervision.h"
#include "safety_states.h"
#include "safety_selftest.h"
#include <soc.h>
#include <zephyr/sys/printk.h>

extern "C" void spin_data_before_start(void)
{
    for (int i = 0; i < safety_trips_watchdog_count(); i++)
    {
        uint8_t adc, channel;
        safety_trips_watchdog_channel(i, &adc, &channel);
        spin.data.ensureChannel(static_cast<adc_t>(adc), channel);
        /* A software-triggered ADC converts only when the application asks:
         * give it its HRTIM trigger so the watchdog always sees data. */
        if (adc_get_trigger_source(adc) == software)
        {
            spin.data.configureTriggerSource(static_cast<adc_t>(adc), TRIG_PWM);
            printk("safety: ADC%u switched to its HRTIM trigger so its watchdog always sees data\n", adc);
        }
    }
}

extern "C" bool safety_data_started(void)
{
    return spin.data.started();
}

/* Started by the Safety API only when the first PWM output comes before data
 * acquisition: externally-triggered dispatch with DMA interrupts off, so the
 * ADCs and the DMA run on their own and the CPU is never interrupted (a DMA
 * interrupt per conversion sequence at the PWM rate would starve it). */
extern "C" int safety_data_start(void)
{
    printk("safety: data acquisition started by the Safety API for its ADC watchdogs (DMA, no interrupt). "
           "An application that reads sensors must start acquisition before its first PWM output.\n");
    return spin.data.startUnattended();
}

/* ---- Sensor plausibility (ADR 0009, #21; IEC 60730-1 H.2 7.2) -------------
 * Runs in the control-task interrupt: arrays and DataAPI::peekAdcChannel only
 * (the shield sensor API may read NVS on first use). A current sensor watched
 * by an ADC watchdog whose reading does not change for
 * CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS control periods while the PWM runs is
 * stuck (a real current sensor always shows a little noise). A sensor with no
 * value for as long while the PWM runs latches too: nothing dispatches the
 * data, so the check would otherwise be off without saying so (#23). */

#define MAX_WATCHED 8
static uint16_t last_raw[MAX_WATCHED];
static uint32_t same_count[MAX_WATCHED];
static uint32_t none_count[MAX_WATCHED];
static uint8_t rank_of[MAX_WATCHED];
static int next_sensor;

/* One current sensor per period, in turn, on its raw sample (an unchanged
 * reading needs no conversion): about 0.5 us instead of 2.5 us per sensor
 * every period (#27). Each check counts as many periods as there are sensors
 * in the turn, so CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS stays in periods. */
extern "C" void safety_plausibility_period(void)
{
    static int8_t currents = -1;   /* current sensors watched by an ADC watchdog */
    static int8_t index_of[MAX_WATCHED];
    if (currents < 0)
    {
        int n = safety_trips_watchdog_count();
        currents = 0;
        for (int i = 0; i < n && i < MAX_WATCHED; i++)
        {
            if (safety_trips_watchdog_is_current(i)) index_of[currents++] = (int8_t)i;
        }
    }
    if (currents == 0)
    {
        return;
    }
    bool running = (HRTIM1->sCommonRegs.OENR & 0xFFFU) != 0U;
    int k = next_sensor;
    next_sensor = (next_sensor + 1) % currents;
    int i = index_of[k];
    uint8_t adc, channel;
    safety_trips_watchdog_channel(i, &adc, &channel);
    if (rank_of[k] == 0)
    {
        rank_of[k] = spin.data.getAdcChannelRank(static_cast<adc_t>(adc), channel);
    }
    uint16_t v = spin.data.peekAdcRaw(static_cast<adc_t>(adc), rank_of[k]);
    if (!running)
    {
        same_count[k] = 0;
        none_count[k] = 0;
        last_raw[k] = v;
        return;
    }
    if (v == 0xFFFFU)
    {
        /* No value yet (the first periods after a start), or none at all. */
        uint32_t before = none_count[k];
        none_count[k] += (uint32_t)currents;
        same_count[k] = 0;
        last_raw[k] = v;
        if (before < CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS && none_count[k] >= CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS &&
            safety_supervision_get_fault() == SAFETY_FAULT_NONE)
        {
            safety_supervision_latch(SAFETY_FAULT_SELFTEST, SAFETY_SELFTEST_NO_DATA | ((uint32_t)i << 8));
            safety_emergency_stop();
        }
        return;
    }
    none_count[k] = 0;
    if (v == last_raw[k])
    {
        uint32_t before = same_count[k];
        same_count[k] += (uint32_t)currents;
        if (before < CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS && same_count[k] >= CONFIG_OWNTECH_SAFETY_FROZEN_PERIODS &&
            safety_supervision_get_fault() == SAFETY_FAULT_NONE)
        {
            safety_supervision_latch(SAFETY_FAULT_SELFTEST, SAFETY_SELFTEST_FROZEN | ((uint32_t)i << 8));
            safety_emergency_stop();
        }
    }
    else
    {
        same_count[k] = 0;
        last_raw[k] = v;
    }
}
