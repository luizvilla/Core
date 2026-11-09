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
 * @brief  Boost converter and single-phase inverter control on the uSolarVerter shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>
#include <zephyr/kernel.h>

/*--------------CMSIS----------------------------------------- */
#include <arm_math.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "app/control_step.h"
#include <math.h>
#include "ScopeMimicry.h"

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

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* Timing, controller tuning and the control logic itself live in app/control_step. */

uint8_t received_serial_char;

/* Measure variables */

static float32_t v_low_value;
static float32_t v_ac_value;
static float32_t v_ac_rms_value;
static float32_t v_dc_bus_value;
static float32_t i_low_1_value;
// static float32_t i_low_2_value;
static float32_t i_ac_value;
/* A new VAC sample arrived in this critical tick. */
static bool v_ac_updated;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

static BoostDutyCycles boost_duty;
static uint16_t leg1_low_deadtime_ns = 100;
// static uint16_t leg2_low_deadtime_ns = 100;
// static constexpr uint16_t deadtime_step_ns = 10;
// static constexpr uint16_t deadtime_min_ns = 40;

/* Shared boost output capacitor feeding the DC/AC stage (VDCBus). */
static float32_t voltage_reference = app::initial_voltage_reference;
/* Boost, inverter and grid observer, with the state kept between control steps. */
static ControlState control;

/* One nominal 50 Hz period at the 10 kHz critical task rate. */
static constexpr uint32_t v_ac_rms_window_sample_count = 200U;
static float32_t grid_angle;
static float32_t v_ac_rms_square_sum;
static uint32_t v_ac_rms_sample_count;

/* ScopeMimicry */
/* 12 KiB capture: 256 samples of 12 channels, half before and half after the trigger: Sync rising
 * at 2 kHz; in the open-loop stage, the bridge start at 2 kHz; in CONNECT/INJECT, the relay command
 * rising at 10 kHz.
 * Keep the byte count within ScopeMimicry's uint16_t buffer-size/dump limits. */
static constexpr uint16_t scope_buffer_sample_count = 256U;
static constexpr uint16_t scope_channel_count = 12U;
static constexpr uint16_t scope_sample_divider = 5U;
static_assert(scope_buffer_sample_count * scope_channel_count * sizeof(float32_t) <= UINT16_MAX,
              "Scope capture exceeds ScopeMimicry's buffer-size limit");
static ScopeMimicry scope(scope_buffer_sample_count, scope_channel_count);
static bool is_scope_downloading;
static uint16_t scope_sample_tick;
/* ScopeMimicry requires persistent float channels, including flags and enums. */
static float32_t scope_v_alpha;
static float32_t scope_v_beta;
static float32_t scope_v_d;
static float32_t scope_v_q;
static float32_t scope_omega;
static float32_t scope_sync;
static float32_t scope_relay_closed;
static float32_t scope_inverter_mode;
static float32_t scope_bridge_duty;   /* applied bridge duty (0 when the bridge is off) */
static float32_t scope_bridge_on;

/*--------------------------------------------------------------- */

/* Mode requests are applied to the controllers by the critical task. */
static boost_mode mode = IDLE;
/* Inverter bench stage (docs/testing/bench/inverter-session.md) and INJECT current. */
static InverterStage inverter_stage = INV_OFF;
static float32_t inject_current = 0.0F;
static constexpr float32_t inject_current_step = 0.01F;  /* A peak per key press */
/* Grid relay on GPIO 22: command, and feedback once held for the contact time.
 * Polarity: set = closed (check on the dead-bus relay test, session step R). */
static constexpr uint8_t relay_pin = 22U;
static bool relay_command = false;
/* Manual relay test (menu 'R'): close the relay alone, dead bench only (session pre-check L2-INV-10.e).
 * Accepted only with the inverter OFF, the boost IDLE, the bridge stopped and no grid voltage. */
static bool relay_test = false;
static constexpr float32_t relay_test_vac_max = 20.0F;  /* V RMS: above this, a grid is present */
static bool relay_closed = false;
static uint16_t relay_hold_ticks = 0U;
/* Critical task load: longest run since the last telemetry line, in µs (period 100 µs). */
static volatile uint32_t critical_max_us = 0U;
/* Critical task calls, to check the real call rate against the 10 kHz the controllers assume. */
static volatile uint32_t critical_calls = 0U;
/* Longest interval between two critical task starts (µs, expected 100), and control periods without
 * a new VAC sample (the last value is held), since the last telemetry line. */
static volatile uint32_t critical_gap_max_us = 0U;
static uint32_t critical_prev_start = 0U;
static volatile uint32_t vac_stale_count = 0U;
static uint32_t rate_prev_calls = 0U;
static int64_t rate_prev_ms = 0;
/* Menu 'v': 10 kHz capture in CONNECT/INJECT (default) or 2 kHz as in the other stages. */
static bool scope_fast_connect = true;
/* Menu 'e': step 1 with the library's settle counter running (diagnostic for the fault after 'g'). */
static bool settle_count_test = false;
/* Menu 'z': protection test, one control period sees an artificial AC current above the bench trip
 * (app::bench_inverter_current_trip); the scope's IAC channel shows it. */
static volatile uint16_t trip_test_countdown = 0U;  /* periods left before the artificial sample (0: none) */
static constexpr uint16_t trip_test_delay_ticks = 1000U;  /* 100 ms: fills the scope's pre-trigger half at 2 or 10 kHz */
static bool scope_trigger_on_trip = false;  /* the capture waits for the bench trip (armed by 'z') */
static constexpr float32_t trip_test_current = 3.0F;  /* A */
/* Same, per part of the task: sensors, control step, PWM and relay application, scope. */
static volatile uint32_t part_max_us[4] = {0U, 0U, 0U, 0U};
static inline void notePart(int part, uint32_t from, uint32_t to)
{
    const uint32_t us = k_cyc_to_us_floor32(to - from);
    if (us > part_max_us[part]) part_max_us[part] = us;
}

/* The burst generator is running on leg 1. */
static bool burst_running = false;
/* Stage neutral bridge duty for one control tick before enabling PWM. */
static bool inverter_pwm_staged = false;
/* Bridge outputs started (start/stop are applied on changes only, not every tick). */
static bool bridge_outputs_on = false;

/* Bench calibration of this board (2026-11-09), applied on top of the default conversion:
 * - VDCBus: meter 99.54 V read 96.4 V, meter 399.4 V read 388.5 V (default conversion), so
 *   V = 1.02643 · reading + 0.592 V. (A first fit used 357.4 V at 399.4 V, taken from a capture at
 *   another bus voltage, and read 435 V at 399.4 V.)
 * - IAC: 0.30 A read with no current (firmware capture, relay open). */
static constexpr float32_t vdc_bus_cal_gain = 1.02643F;
static constexpr float32_t vdc_bus_cal_offset = 0.592F;
static constexpr float32_t i_ac_cal_offset = -0.30F;

/** @brief Compose the bench calibration with the sensors' default linear conversion. */
static void applyBenchCalibration()
{
    const float32_t g_bus = shield.sensors.retrieveStoredParameterValue(VDCBus, gain);
    const float32_t o_bus = shield.sensors.retrieveStoredParameterValue(VDCBus, offset);
    shield.sensors.setConversionParametersLinear(VDCBus, g_bus * vdc_bus_cal_gain,
                                                 o_bus * vdc_bus_cal_gain + vdc_bus_cal_offset);
    const float32_t g_iac = shield.sensors.retrieveStoredParameterValue(IAC, gain);
    const float32_t o_iac = shield.sensors.retrieveStoredParameterValue(IAC, offset);
    shield.sensors.setConversionParametersLinear(IAC, g_iac, o_iac + i_ac_cal_offset);
}

/* Menu 'f': trigger the capture now, whatever the stage (e.g. a synchronization that never comes). */
static volatile bool scope_force_trigger = false;

bool scope_trigger()
{
    if (scope_force_trigger)
    {
        scope_force_trigger = false;
        return true;
    }
    /* Grid stages: capture the relay closing, or an inverter fault before it; open loop: the bridge
     * start; otherwise the synchronization. */
    if (scope_trigger_on_trip) return control.bench_tripped;
    if (inverter_stage == INV_CONNECT || inverter_stage == INV_INJECT)
        return scope_relay_closed > 0.5F || scope_inverter_mode == static_cast<float32_t>(VSI_FAULT);
    if (inverter_stage == INV_OPEN_LOOP) return scope_bridge_on > 0.5F;
    return scope_sync > 0.5F;
}

void dump_scope_datas(ScopeMimicry &scope)
{
    printk("begin record\n");
    scope.reset_dump();
    while (scope.get_dump_state() != finished)
    {
        printk("%s", scope.dump_datas());
        task.suspendBackgroundUs(200);
    }
    printk("end record\n");
}

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the boost legs and D/E inverter
 *  - Initializes the power shield sensors
 *  - Initializes the boost and inverter controllers and scope
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Only boost leg 1 is enabled; boost leg 2 is disabled. */
    shield.power.initBoost(LEG1_LOW);
    // shield.power.initBoost(LEG2_LOW);
    shield.power.setDeadTime(LEG1_LOW, leg1_low_deadtime_ns, leg1_low_deadtime_ns);
    // shield.power.setDeadTime(LEG2_LOW, leg2_low_deadtime_ns, leg2_low_deadtime_ns);
    /* Unipolar DC/AC bridge, initially at neutral duty. Leg A (follows D) must be the leg
     * driving L: on the uSolarVerter, LEG2_HIGH drives L (FB1/L1) and LEG1_HIGH drives N
     * (FB2/L3). With LEG1_HIGH as leg A the output was inverted, 180° from the grid (bench
     * 2026-11-09). Duty updates go to leg A (setInverterDutyCycle below). */
    if (shield.power.initInverterUnipolar(LEG2_HIGH, LEG1_HIGH) != 0)
    {
        printk("inverter initialization failed\n");
        return;
    }
    shield.power.setDutyCycleMin(LEG1_LOW, app::duty_cycle_min);
    shield.power.setDutyCycleMin(LEG1_HIGH, app::duty_cycle_min);
    shield.power.setDutyCycleMin(LEG2_HIGH, app::duty_cycle_min);
    shield.power.setDutyCycleMax(LEG1_HIGH, app::duty_cycle_max_high_side);
    shield.power.setDutyCycleMax(LEG2_HIGH, app::duty_cycle_max_high_side);
    shield.power.setDutyCycleMax(LEG1_LOW, app::duty_cycle_max);
    // shield.power.setDutyCycleMax(LEG2_LOW, app::duty_cycle_max);
    
    /* Current-driven burst on boost leg 1 only (ADR 0003). */
    shield.power.initBurstMode(LEG1_LOW);

    shield.sensors.enableSensor(VLow, ADC_1);
    shield.sensors.enableSensor(VAC, ADC_1);
    shield.sensors.enableSensor(VDCBus, ADC_1);
    shield.sensors.enableSensor(ILow1, ADC_2);
    // shield.sensors.enableSensor(ILow2, ADC_2);
    shield.sensors.enableSensor(IAC, ADC_2);
    applyBenchCalibration();  /* before the data acquisition starts (critical task) */

    spin.gpio.configurePin(relay_pin, OUTPUT);
    spin.gpio.resetPin(relay_pin);  /* relay open at start-up */
    const int8_t init_result = controlInit(control);
    if (init_result == -1)
    {
        printk("controller initialization failed\n");
        return;
    }
    if (init_result != 0)
    {
        printk("invalid boost burst, protection or gain schedule\n");
        return;
    }
    scope.connectChannel(v_ac_value, "VAC_V");
    scope.connectChannel(scope_v_alpha, "V_alpha_V");
    scope.connectChannel(scope_v_beta, "V_beta_V");
    scope.connectChannel(scope_v_d, "V_d_V");
    scope.connectChannel(scope_v_q, "V_q_V");
    scope.connectChannel(i_ac_value, "IAC_A");
    scope.connectChannel(scope_omega, "omega_rad_s");
    scope.connectChannel(scope_sync, "Sync");
    scope.connectChannel(scope_relay_closed, "relay_cmd");
    scope.connectChannel(scope_inverter_mode, "inverter_mode");
    scope.connectChannel(v_dc_bus_value, "VDCBus_V");
    scope.connectChannel(scope_bridge_duty, "bridge_duty");
    scope.set_trigger(&scope_trigger);
    scope.set_delay(0.5F);
    scope.start();

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, app::control_task_period);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This tasks implements a minimalistic USB serial interface to control
 * the boost converter and inverter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ---- MENU microinverter ----       |\n"
               "|     press i : idle mode                |\n"
               "|     press p : startup (0.25 A clamp)   |\n"
               "|     press b : bus regulation mode      |\n"
               "|     press m : MPPT mode                |\n"
               "|  inverter bench session (DC source):  |\n"
               "|     press o : step 0 open loop 230 V   |\n"
               "|     press c : step 1 PWM_SETTLE hold   |\n"
               "|     press g : step 2 connect (relay)   |\n"
               "|     press n : step 3 inject current    |\n"
               "|     press j / k : inject current +/-   |\n"
               "|     press x : inverter OFF (relay open)|\n"
               "|     press R : relay test (dead bench)  |\n"
               "|     press u : voltage reference UP     |\n"
               "|     press d : voltage reference DOWN   |\n"
               // "|     press w : deadtime UP              |\n"
               // "|     press s : deadtime DOWN            |\n"
               "|     press r : retrieve scope data      |\n"
               "|     press q : restart scope acquisition|\n"
               "|     press f : force scope trigger now  |\n"
               "|     press v : CONNECT scope 10k/2k Hz  |\n"
               "|     press e : step 1 settle counter on |\n"
               "|     press z : protection test (3 A)    |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLE;
        inverter_stage = INV_OFF;
        relay_test = false;
        break;
    case 'p':
        printk("startup: converter 1, current clamped\n");
        mode = STARTUP;
        inverter_stage = INV_OFF;
        break;
    case 'b':
        printk("VDCBus regulation mode\n");
        mode = BUSREG;
        inverter_stage = INV_OFF;
        break;
    case 'm':
        printk("MPPT current mode\n");
        mode = MPPT;
        inverter_stage = INV_OFF;
        break;
    case 'o':
        printk("inverter step 0: open loop to 230 V RMS (relay open)\n");
        inverter_stage = INV_OPEN_LOOP;
        relay_test = false;
        break;
    case 'c':
        printk("inverter step 1: PWM_SETTLE hold (relay open)\n");
        inverter_stage = INV_SETTLE;
        scope_trigger_on_trip = false;
        relay_test = false;
        break;
    case 'g':
        printk("inverter step 2: settle %u periods, close the relay, zero current\n",
               static_cast<unsigned int>(app::inverter_settle_cycles));
        inverter_stage = INV_CONNECT;
        scope_trigger_on_trip = false;
        relay_test = false;
        scope.start();  /* re-arm: capture the relay closing (or a fault) of this stage */
        break;
    case 'n':
        printk("inverter step 3: inject %.3f A (d axis, peak)\n", (double)inject_current);
        inverter_stage = INV_INJECT;
        scope_trigger_on_trip = false;
        relay_test = false;
        scope.start();  /* re-arm: capture this stage's start (or a fault) */
        break;
    case 'j':
        inject_current += inject_current_step;
        if (inject_current > app::inject_current_max) inject_current = app::inject_current_max;
        printk("inject current: %.3f A peak\n", (double)inject_current);
        break;
    case 'k':
        inject_current -= inject_current_step;
        if (inject_current < -app::inject_current_max) inject_current = -app::inject_current_max;
        printk("inject current: %.3f A peak\n", (double)inject_current);
        break;
    case 'x':
        printk("inverter OFF: disconnect, relay open\n");
        inverter_stage = INV_OFF;
        relay_test = false;
        break;
    case 'R':
        if (relay_test)
        {
            relay_test = false;
            printk("relay test: open\n");
        }
        else if (inverter_stage != INV_OFF || mode != IDLE || control.bridge_pwm_applied)
        {
            printk("relay test refused: inverter OFF (x) and boost IDLE (i) first\n");
        }
        else if (v_ac_rms_value > relay_test_vac_max || fabsf(v_ac_value) > relay_test_vac_max * 1.5F)
        {
            printk("relay test refused: grid voltage present (%.1f V RMS); unplug mains first\n",
                   (double)v_ac_rms_value);
        }
        else
        {
            relay_test = true;
            printk("relay test: CLOSED (press R again to open; any stage key opens it)\n");
        }
        break;
    case 'u':
        // Keep the bus target inside the gain schedule's range, well below the full-burst level.
        if (voltage_reference + 1.0F <= app::boost_schedule_vbus_max) voltage_reference += 1.0F;
        break;
    case 'd':
        voltage_reference -= 1.0F;
        if (voltage_reference < 1.0F) voltage_reference = 1.0F;
        break;
    // case 'w':
    //     leg2_low_deadtime_ns += deadtime_step_ns;
    //     shield.power.setDeadTime(LEG2_LOW, leg2_low_deadtime_ns, leg2_low_deadtime_ns);
    //     printk("LEG2_LOW deadtime: %u ns\n", leg2_low_deadtime_ns);
    //     break;
    // case 's':
    //     if (leg2_low_deadtime_ns >= deadtime_step_ns)
    //     {
    //         leg2_low_deadtime_ns -= deadtime_step_ns;
    //     }
    //     else
    //     {
    //         leg2_low_deadtime_ns = deadtime_min_ns;
    //     }
    //     shield.power.setDeadTime(LEG2_LOW, leg2_low_deadtime_ns, leg2_low_deadtime_ns);
    //     printk("LEG2_LOW deadtime: %u ns\n", leg2_low_deadtime_ns);
    //     break;
    case 'r':
        is_scope_downloading = true;
        break;
    case 'q':
        scope.start();
        printk("scope acquisition restarted\n");
        break;
    case 'z':
        scope_trigger_on_trip = true;
        scope.start();  /* re-arm: the capture triggers on the trip */
        trip_test_countdown = trip_test_delay_ticks;
        printk("protection test: in 100 ms, one period at %.1f A (trip above %.1f A); scope armed on the trip\n",
               (double)trip_test_current, (double)app::bench_inverter_current_trip);
        break;
    case 'e':
        settle_count_test = !settle_count_test;
        printk("step 1 settle counter: %s\n", settle_count_test ? "RUNNING (diagnostic)" : "off (normal)");
        break;
    case 'v':
        scope_fast_connect = !scope_fast_connect;
        printk("scope in CONNECT/INJECT: %s\n", scope_fast_connect ? "10 kHz" : "2 kHz");
        break;
    case 'f':
        scope_force_trigger = true;
        printk("scope trigger forced: wait 70 ms, then r\n");
        break;
    case 't':
        /* Former manual relay release: now the inverter OFF stage, which opens the relay safely. */
        inverter_stage = INV_OFF;
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
    if (is_scope_downloading)
    {
        dump_scope_datas(scope);
        is_scope_downloading = false;
        task.suspendBackgroundMs(100);
        return;
    }

    if (mode == IDLE && inverter_stage == INV_OFF) spin.led.turnOff();
    else spin.led.turnOn();

    if (control.boost.isTripped()) printk("boost tripped: press i (idle) to clear\n");
    if (control.bench_tripped) printk("inverter bench current trip: press x (inverter OFF) to clear\n");
    /* ILow1, VLow, duty1, VAC, VACrms, angle, IAC, VDCBus, mode, burst skip. */
    printk("%7.3f:", (double)i_low_1_value);
    // printk("%7.3f:", (double)i_low_2_value);
    printk("%7.3f:", (double)v_low_value);
    printk("%7.3f:", (double)boost_duty.leg1);
    // printk("%7.3f:", (double)boost_duty.leg2);
    printk("%7.3f:", (double)v_ac_value);
    printk("%7.3f:", (double)v_ac_rms_value);
    printk("%7.3f:", (double)grid_angle);
    printk("%7.3f:", (double)i_ac_value);
    printk("%7.3f:", (double)v_dc_bus_value);
    printk("%u:%u:", static_cast<unsigned int>(mode),
           static_cast<unsigned int>(control.boost.getBurstSkip()));
    /* Inverter: stage, library mode, relay command/feedback, Id, Iq (A), P (W), Q (var), Id ref. */
    const dqo_t idq = control.inverter.getIdq();
    const dqo_t pq = control.inverter.getPower();
    const uint32_t load_us = critical_max_us;
    critical_max_us = 0U;
    const uint32_t calls = critical_calls;
    const int64_t now_ms = k_uptime_get();
    const uint32_t rate_hz = now_ms > rate_prev_ms ?
        static_cast<uint32_t>((static_cast<int64_t>(calls - rate_prev_calls) * 1000) / (now_ms - rate_prev_ms)) : 0U;
    rate_prev_calls = calls;
    rate_prev_ms = now_ms;
    const uint32_t gap_max = critical_gap_max_us;
    critical_gap_max_us = 0U;
    const uint32_t vac_stale = vac_stale_count;
    vac_stale_count = 0U;
    printk("load us: total %u, sensors %u, control %u, pwm %u, scope %u, rate %u Hz, gap max %u us, vac stale %u\n",
           static_cast<unsigned int>(load_us),
           static_cast<unsigned int>(part_max_us[0]), static_cast<unsigned int>(part_max_us[1]),
           static_cast<unsigned int>(part_max_us[2]), static_cast<unsigned int>(part_max_us[3]),
           static_cast<unsigned int>(rate_hz), static_cast<unsigned int>(gap_max),
           static_cast<unsigned int>(vac_stale));
    for (int k = 0; k < 4; ++k) part_max_us[k] = 0U;
    /* Synchronization criterion, as the inverter judges it (one-period averages, #7) and the grid verdict. */
    const dqo_t vdq_avg = control.inverter.getVdqAverage();
    printk("sync: Vd_avg %.1f (293-358) Vq_avg %.2f (|.|<3.25) w_avg %.2f (298.5-329.9) window %u grid_ok %u trip %u fault %u\n",
           (double)vdq_avg.d, (double)vdq_avg.q, (double)control.inverter.getwAverage(),
           static_cast<unsigned int>(control.inverter.isSyncWindowFull()),
           static_cast<unsigned int>(control.grid_observer.canOperate()),
           static_cast<unsigned int>(control.grid_observer.getTripReason()),
           static_cast<unsigned int>(control.inverter.getFaultReason()));
    printk("%u:%u:%u%u:%7.3f:%7.3f:%7.1f:%7.1f:%6.3f:%u\n", static_cast<unsigned int>(inverter_stage),
           static_cast<unsigned int>(control.inverter.getMode()), static_cast<unsigned int>(relay_command),
           static_cast<unsigned int>(relay_closed), (double)idq.d, (double)idq.q, (double)pq.d, (double)pq.q,
           (double)inject_current, static_cast<unsigned int>(load_us));
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - It runs inverter control and the boost hot loop every tick
 *  - It runs the boost voltage/ramp or MPPT cold loop every tenth tick
 *  - Boost leg 1 skips PWM periods with the burst generator when the controller asks
 *  - It updates the PWM signals
 */
void loop_critical_task()
{
    const uint32_t critical_start = k_cycle_get_32();
    if (critical_calls != 0U)
    {
        const uint32_t gap_us = k_cyc_to_us_floor32(critical_start - critical_prev_start);
        if (gap_us > critical_gap_max_us) critical_gap_max_us = gap_us;
    }
    critical_prev_start = critical_start;
    ++critical_calls;
    /*----- Read sensors, holding the last valid value ----- */
    meas_data = shield.sensors.getLatestValue(ILow1);
    if (meas_data != NO_VALUE) i_low_1_value = meas_data;

    meas_data = shield.sensors.getLatestValue(VLow);
    if (meas_data != NO_VALUE) v_low_value = meas_data;

    v_ac_updated = false;
    meas_data = shield.sensors.getLatestValue(VAC);
    if (meas_data != NO_VALUE)
    {
        v_ac_updated = true;
        v_ac_value = meas_data;
    }
    else
    {
        ++vac_stale_count;
    }

    // meas_data = shield.sensors.getLatestValue(ILow2);
    // if (meas_data != NO_VALUE) i_low_2_value = meas_data;

    meas_data = shield.sensors.getLatestValue(IAC);
    if (meas_data != NO_VALUE) i_ac_value = meas_data;
    if (trip_test_countdown != 0U && --trip_test_countdown == 0U)
    {
        i_ac_value = trip_test_current;
    }

    meas_data = shield.sensors.getLatestValue(VDCBus);
    if (meas_data != NO_VALUE)
    {
        v_dc_bus_value = meas_data;
    }

    const uint32_t t_sensed = k_cycle_get_32();
    notePart(0, critical_start, t_sensed);
    /*----- Control logic (hardware-free, see app/control_step.h) ----- */
    Measurements meas{};
    meas.i_low_1 = i_low_1_value;
    // meas.i_low_2 = i_low_2_value;
    meas.v_low = v_low_value;
    meas.v_dc_bus = v_dc_bus_value;
    meas.v_ac = v_ac_value;
    meas.i_ac = i_ac_value;
    meas.v_ac_updated = v_ac_updated;
    meas.relay_closed = relay_closed;

    OperatorCommands cmd;
    cmd.mode = mode;
    cmd.inverter_stage = inverter_stage;
    cmd.settle_count_test = settle_count_test;
    cmd.inject_current = inject_current;
    cmd.v_bus_ref = voltage_reference;

    const ControlOutputs out = controlStep(control, meas, cmd);
    const uint32_t t_controlled = k_cycle_get_32();
    notePart(1, t_sensed, t_controlled);
    boost_duty = out.boost_duty;
    grid_angle = out.grid_angle;

    /*----- Apply PWM commands ----- */
    if (out.boost_mode_changed)
    {
        shield.power.stop(LEG1_LOW);
        // shield.power.stop(LEG2_LOW);
    }

    if (out.boost_pwm_update)
    {
        shield.power.setDutyCycle(LEG1_LOW, out.boost_duty.leg1);
        // shield.power.setDutyCycle(LEG2_LOW, out.boost_duty.leg2);
        if (out.leg1_enable) shield.power.start(LEG1_LOW);
        else shield.power.stop(LEG1_LOW);
        // if (out.leg2_enable) shield.power.start(LEG2_LOW);
        // else shield.power.stop(LEG2_LOW);
    }

    /* Leg 1 burst: skip out.burst_skip of every out.burst_period PWM periods. */
    if (out.boost_pwm_update && out.leg1_enable && out.burst_skip > 0U)
    {
        shield.power.setBurstMode(out.burst_skip, out.burst_period);
        if (!burst_running)
        {
            shield.power.startBurstMode();
            burst_running = true;
        }
    }
    else if (burst_running)
    {
        shield.power.stopBurstMode();
        burst_running = false;
    }

    /* Apply paired bridge duty; acknowledge only after PWM is enabled. */
    if (out.bridge_enable &&
        shield.power.setInverterDutyCycle(LEG2_HIGH, out.bridge_duty) == 0)
    {
        if (inverter_pwm_staged)
        {
            if (!bridge_outputs_on)
            {
                shield.power.start(LEG1_HIGH);
                shield.power.start(LEG2_HIGH);
                bridge_outputs_on = true;
            }
            control.bridge_pwm_applied = true;
        }
        else
        {
            /* Allow the initial neutral duty to latch before starting outputs. */
            shield.power.stop(LEG1_HIGH);
            shield.power.stop(LEG2_HIGH);
            bridge_outputs_on = false;
            control.bridge_pwm_applied = false;
            inverter_pwm_staged = true;
        }
    }
    else
    {
        if (out.bridge_enable) control.inverter.tripFault(VSI_FAULT_PWM_UPDATE_FAILED);
        if (bridge_outputs_on || inverter_pwm_staged)
        {
            shield.power.stop(LEG1_HIGH);
            shield.power.stop(LEG2_HIGH);
            bridge_outputs_on = false;
        }
        control.bridge_pwm_applied = false;
        inverter_pwm_staged = false;
    }
    control.inverter.setPwmEnabled(control.bridge_pwm_applied && !control.open_loop_active);

    /* Grid relay (GPIO 22): follow the command; report it closed/open once held for the contact time.
     * The manual test only holds while nothing else runs and no grid voltage appears. */
    if (relay_test && (inverter_stage != INV_OFF || mode != IDLE || out.bridge_enable ||
                       fabsf(v_ac_value) > relay_test_vac_max * 1.5F))
    {
        relay_test = false;
    }
    const bool relay_wanted = out.relay_close || relay_test;
    if (relay_wanted != relay_command)
    {
        relay_command = relay_wanted;
        relay_hold_ticks = 0U;
        if (relay_command) spin.gpio.setPin(relay_pin);
        else spin.gpio.resetPin(relay_pin);
    }
    else if (relay_hold_ticks < app::relay_contact_ticks)
    {
        ++relay_hold_ticks;
    }
    else
    {
        relay_closed = relay_command;
    }

    const uint32_t t_applied = k_cycle_get_32();
    notePart(2, t_controlled, t_applied);
    /* Snapshot after this tick's control, grid verdict and PWM application. */
    const clarke_t grid_voltage_ab = control.inverter.getVab();
    const dqo_t grid_voltage_dq = control.inverter.getVdq();
    scope_v_alpha = grid_voltage_ab.alpha;
    scope_v_beta = grid_voltage_ab.beta;
    scope_v_d = grid_voltage_dq.d;
    scope_v_q = grid_voltage_dq.q;
    scope_omega = control.inverter.getw();
    v_ac_rms_value = control.grid_observer.getVoltageRms();
    scope_sync = control.inverter.getSync() ? 1.0F : 0.0F;
    scope_relay_closed = relay_command ? 1.0F : 0.0F;
    scope_inverter_mode = static_cast<float32_t>(control.inverter.getMode());
    scope_bridge_on = control.bridge_pwm_applied ? 1.0F : 0.0F;
    scope_bridge_duty = control.bridge_pwm_applied ? out.bridge_duty : 0.0F;
    /* Grid stages: 10 kHz around the relay closing (25.6 ms window); otherwise 2 kHz. */
    const uint16_t divider = (scope_fast_connect && (inverter_stage == INV_CONNECT || inverter_stage == INV_INJECT)) ?
                             1U : scope_sample_divider;
    if (++scope_sample_tick >= divider)
    {
        scope_sample_tick = 0U;
        if (!is_scope_downloading) scope.acquire();
    }
    const uint32_t t_end = k_cycle_get_32();
    notePart(3, t_applied, t_end);
    const uint32_t critical_us = k_cyc_to_us_floor32(t_end - critical_start);
    if (critical_us > critical_max_us) critical_max_us = critical_us;
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
