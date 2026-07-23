/*
 *
 * Copyright (c) 2024-present LAAS-CNRS
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
 * @brief  This file is the main application code for the open-loop inverter
 *         example. It implements a sinusoidal open-loop PWM strategy that
 *         drives an H-bridge inverter using a local oscillator without any
 *         closed-loop feedback or grid synchronization.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

 /*--------------OWNTECH APIs---------------------------------- */
#include <math.h>
#include <float.h>
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "user_data_objects.h"

/* From control library */
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F

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
/* Reports whether the scope capture should trigger */
bool a_trigger();
/* Clamps a floating-point value inside the requested range */
float32_t saturate(float32_t value, float32_t min, float32_t max);
/* Returns the DC bus voltage, or a fallback before sensing is valid */
float32_t control_bus_voltage();
/* Starts both PWM legs once and records the output state */
void start_pwm_outputs();
/* Stops both PWM legs once and records the output state */
void stop_pwm_outputs();
/* Advances the local oscillator and updates the sine voltage reference */
void update_teaching_sine();
/* Applies clamped complementary duty cycles to the H-bridge legs */
void apply_complementary_duty(float32_t duty);
/* Streams a completed ScopeMimicry capture over the serial console */
void dump_scope_datas(ScopeMimicry &scope_to_dump);
/* Adjusts the local sine voltage amplitude used by the open-loop PWM */
void adjust_amplitude(float32_t step);
/* Registers scope channels and starts capture for the open-loop example */
void setup_scope();
/* Reads sensor values and derives the measured grid voltage */
void read_measurements();
/* Checks both measured currents against the protection threshold */
bool overcurrent_detected();
/* Accumulates squared currents and refreshes the RMS values once per grid period */
void update_rms();
/* Tracks the peak current over a grid period and derives RMS as peak/sqrt(2) */
void update_rms_peak();
/* Tracks an EMA/IIR estimate of the mean-square current and derives RMS continuously */
void update_rms_ema();
/* Tracks V_high min/max/mean and refreshes the bus voltage ripple once per grid period */
void update_voltage_ripple();

/*--------------USER VARIABLES DECLARATIONS------------------- */

enum ConverterState : uint8_t /* Holds the current state of the inverter */
{
    IDLEMODE = 0,  /* Idle mode: stops the converter power */
    POWERMODE = 1, /* Power mode: drives the H-bridge with sine PWM */
    ERRORMODE = 3  /* Error mode: indicates an error condition */
};

/* Control task period in microseconds */
static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
/* Control task period in seconds */
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
/* Fallback DC bus voltage in volts before sensing is valid */
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
/* Grid frequency in hertz */
static constexpr float32_t F0 = 50.0F;
/* Grid pulsation in radians per second */
static constexpr float32_t W0 = 2.0F * PI * F0;
/* Maximum current for overcurrent protection in amps */
static constexpr float32_t MAX_CURRENT = 10.0F;
/* Maximum current for overcurrent protection in amps */
static constexpr float32_t MAX_CURRENT_OPERATION = 2.0F;
/* Number of critical task samples in one grid period (TS * RMS_WINDOW_SAMPLES = 1/F0) */
static constexpr uint32_t RMS_WINDOW_SAMPLES = static_cast<uint32_t>(1.0F / (F0 * TS) + 0.5F);
/* sqrt(2), used to convert a sinusoid peak value into an RMS value */
static constexpr float32_t SQRT2 = 1.41421356F;
/* [s] Time constant of the EMA/IIR mean-square RMS estimator (~1 grid period) */
static constexpr float32_t RMS_EMA_TAU = 0.02F;
/* Discrete EMA gain derived from RMS_EMA_TAU and the control task period */
static constexpr float32_t RMS_EMA_ALPHA = TS / RMS_EMA_TAU;
/* Size of the scope buffer for data recording */
static constexpr uint32_t SCOPE_BUFFER_SIZE = 1024;
/* Number of channels recorded in the scope for diagnostics */
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 21;

/* State of the PWM outputs */
static bool pwm_enable = false;
/* State of scope data downloading */
static bool is_downloading = false;
/* State to trigger the scope capture */
static bool trigger = false;

/* Variable to store the last received serial character */
static uint8_t received_serial_char;
/* Current operating mode of the inverter */
static uint8_t mode = IDLEMODE;
/* Last requested operating mode from the serial interface */
static uint8_t mode_asked = IDLEMODE;

/* [V] Low-side filtered measurement of voltage 1 */
// static float32_t V1_low_value;
/* [V] Low-side filtered measurement of voltage 2 */
// static float32_t V2_low_value;
/* [A] Low-side filtered measurement of current 1 */
// static float32_t I1_low_value;
/* [A] Low-side filtered measurement of current 2 */
// static float32_t I2_low_value;
/* [A] High-side raw measurement of current */
// static float32_t I_high;
/* [V] High-side raw measurement of voltage */
// static float32_t V_high;
/* [V] Further filtered high-side voltage for control */
static float32_t V_high_filt;
/* [V] Measured grid voltage from the difference of low-side measurements */
static float32_t Vgrid_meas;
/* [V] Temporary variable for storing sensor measurements */
// static float32_t meas_data;

/* [A] RMS value of current 1, refreshed once per grid period */
static float32_t I1_rms;
/* [A] RMS value of current 2, refreshed once per grid period */
static float32_t I2_rms;
/* [A^2] Running sum of squares of I1_low_value over the current grid period */
static float32_t I1_sum_sq;
/* [A^2] Running sum of squares of I2_low_value over the current grid period */
static float32_t I2_sum_sq;
/* Number of samples accumulated in the current RMS window */
static uint32_t rms_sample_count;

/* [A] RMS value of current 1 estimated as peak/sqrt(2), refreshed once per grid period */
static float32_t I1_rms_peak;
/* [A] RMS value of current 2 estimated as peak/sqrt(2), refreshed once per grid period */
static float32_t I2_rms_peak;
/* [A] Running peak (max absolute value) of I1_low_value over the current grid period */
static float32_t I1_peak_running;
/* [A] Running peak (max absolute value) of I2_low_value over the current grid period */
static float32_t I2_peak_running;
/* Number of samples accumulated in the current peak-detection window */
static uint32_t peak_sample_count;

/* [A] RMS value of current 1 estimated from an EMA of I1_low_value^2, updated every cycle */
static float32_t I1_rms_ema;
/* [A] RMS value of current 2 estimated from an EMA of I2_low_value^2, updated every cycle */
static float32_t I2_rms_ema;
/* [A^2] EMA of I1_low_value^2 */
static float32_t I1_meanSq_ema;
/* [A^2] EMA of I2_low_value^2 */
static float32_t I2_meanSq_ema;

/* [V] Peak-to-peak ripple of V_high over the last grid period */
static float32_t Vhigh_ripple_pp;
/* [%] Peak-to-peak ripple of V_high relative to its mean over the last grid period */
static float32_t Vhigh_ripple_pct;
/* [V] Mean of V_high over the current grid period window */
static float32_t Vhigh_mean;
/* [V] Running minimum of V_high in the current grid period window */
static float32_t Vhigh_min_running = FLT_MAX;
/* [V] Running maximum of V_high in the current grid period window */
static float32_t Vhigh_max_running = -FLT_MAX;
/* [V] Running sum of V_high in the current grid period window */
static float32_t Vhigh_sum;
/* Number of samples accumulated in the current voltage ripple window */
static uint32_t vripple_sample_count;

/* [V] Amplitude of the local teaching sine wave */
static float32_t Mp_AC_source = 0.0F;
/* [V] Amplitude of the local teaching sine wave */
static float32_t Mp = 0.0F;
/* [rad] Phase angle of the local teaching sine wave */
static float32_t phi_AC_source;
/* [rad] Phase angle of the reference duty cycle sine wave */
static float32_t phi_m;
/* [No unit] Instantaneous value of the local teaching sine wave */
static float32_t sine;
/* [No unit] Instantaneous value of the reference duty cycle sine wave */
static float32_t sine_modulation;
/* [V] Instantaneous local grid voltage from the teaching sine */
static float32_t local_vgrid;
/* [V] Instantaneous local grid voltage from the teaching sine */
static float32_t local_modulation;

/* [No unit] Duty cycle computed from the local sine and DC bus voltage */
static float32_t delta_duty_cycle = 0.5F;
/* [No unit] Duty cycle for leg 1 of the H-bridge */
static float32_t duty_cycle_1 = 0.5F;
/* [No unit] Duty cycle for leg 2 of the H-bridge */
static float32_t duty_cycle_2 = 0.5F;
/* [No unit] Scope variable for the current operating mode */
static float32_t state_mode_scope;
/* Counter for the number of critical task iterations */
static uint32_t critical_task_counter;

/* First-order low-pass filter for the high-side voltage measurement */
static LowPassFirstOrderFilter vHighFilter(TS, 0.1F);
/* ScopeMimicry instance for recording control variables and diagnostics */
static ScopeMimicry scope(SCOPE_BUFFER_SIZE, SCOPE_CHANNEL_COUNT);

/* Synchronization variables */
static uint32_t dac_value;
/*--------------------------------------------------------------- */

/**********************  SUPPORT FUNCTIONS  ***************************/

/**
 * @brief Reports whether the scope capture should trigger.
 */
bool a_trigger()
{
    return trigger;
}

/**
 * @brief Clamps a floating-point value inside the requested range.
 */
float32_t saturate(float32_t value, float32_t min, float32_t max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

/**
 * @brief Returns the measured DC bus voltage, or a fallback before sensing is valid.
 */
float32_t control_bus_voltage()
{
    if (V_high_filt > 1.0F) {
        return V_high_filt;
    }
    return DC_BUS_FALLBACK;
}

/**
 * @brief Starts both PWM legs once and records the output state.
 */
void start_pwm_outputs()
{
    if (!pwm_enable) {
        shield.power.start(ALL);
        pwm_enable = true;
    }
}

/**
 * @brief Stops both PWM legs once and records the output state.
 */
void stop_pwm_outputs()
{
    if (pwm_enable) {
        shield.power.stop(ALL);
        pwm_enable = false;
    }
}

/**
 * @brief Advances the local teaching oscillator and updates the sine voltage reference.
 */
void update_teaching_sine()
{
    phi_AC_source = ot_modulo_2pi(phi_AC_source + W0 * TS);
    phi_m = ot_modulo_2pi(phi_m + W0 * TS);
    sine = ot_sin(phi_AC_source);
    sine_modulation = ot_sin(phi_m);
    delta_duty_cycle = 0.5F + ( Mp * sine_modulation / 2.0F );
    dac_value = (0.52F + ( Mp_AC_source * sine / 2.0F )) * 4000;
}

/**
 * @brief Applies clamped complementary duty cycles to the H-bridge legs.
 */
void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = saturate(duty, DUTY_MIN, DUTY_MAX);
    duty_cycle_2 = saturate(1.0F - duty, DUTY_MIN, DUTY_MAX);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

/**
 * @brief Streams a completed ScopeMimicry capture over the serial console.
 */
void dump_scope_datas(ScopeMimicry &scope_to_dump)
{
    scope_to_dump.reset_dump();
    printk("begin record\n");
    while (scope_to_dump.get_dump_state() != finished) {
        printk("%s", scope_to_dump.dump_datas());
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

/**
 * @brief Adjusts the local sine voltage amplitude used by the open-loop PWM.
 */
void adjust_amplitude_converter(float32_t step)
{
    Mp =
        saturate(Mp + step, 0.0F, 1.0F);
}

/**
 * @brief Adjusts the local sine voltage amplitude used by the open-loop PWM.
 */
void adjust_amplitude_ACsource(float32_t step)
{
    Mp_AC_source =
        saturate(Mp_AC_source + step, 0.0F, 1.0F);
}

/**
 * @brief Adjusts the local sine voltage phase used by the open-loop PWM.
 */
void adjust_phase_converter(float32_t step)
{
    phi_m =
        ot_modulo_2pi(phi_m + step);
}

/**
 * @brief Adjusts the local sine voltage phase used by the open-loop PWM.
 */
void adjust_phase_ACsource(float32_t step)
{
    phi_AC_source =
        ot_modulo_2pi(phi_AC_source + step);
}

/**
 * @brief Registers scope channels and starts capture for the open-loop example.
 */
void setup_scope()
{
    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(I1_rms, "I1_rms");
    scope.connectChannel(I2_rms, "I2_rms");
    scope.connectChannel(I1_rms_peak, "I1_rms_peak");
    scope.connectChannel(I2_rms_peak, "I2_rms_peak");
    scope.connectChannel(I1_rms_ema, "I1_rms_ema");
    scope.connectChannel(I2_rms_ema, "I2_rms_ema");
    scope.connectChannel(Vhigh_ripple_pp, "Vhigh_ripple_pp");
    scope.connectChannel(Vhigh_ripple_pct, "Vhigh_ripple_pct");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    scope.connectChannel(V_high_filt, "Vdc");
    scope.connectChannel(local_vgrid, "local_vgrid");
    scope.connectChannel(local_modulation, "local_modulation");
    scope.connectChannel(sine, "sine");
    scope.connectChannel(sine_modulation, "sine_modulation");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
    scope.connectChannel(state_mode_scope, "state");
    scope.connectChannel(I_high_value, "state");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
}

/**
 * @brief Reads the latest sensor values and derives the measured grid voltage.
 */
void read_measurements()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high_value = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high_value);
    Vgrid_meas = V1_low_value - V2_low_value;

    I1_low_value = I1_low_value -0.1;
    I2_low_value = I2_low_value -0.15;
}

/**
 * @brief Checks both measured currents against the protection threshold.
 */
bool overcurrent_detected()
{
    return I1_low_value > MAX_CURRENT ||
           I1_low_value < -MAX_CURRENT ||
           I2_low_value > MAX_CURRENT ||
           I2_low_value < -MAX_CURRENT;
}

/**
 * @brief Checks both measured currents against the protection threshold.
 */
bool overcurrent_detected_operation()
{
    return I1_low_value > MAX_CURRENT_OPERATION ||
           I1_low_value < -MAX_CURRENT_OPERATION ||
           I2_low_value > MAX_CURRENT_OPERATION ||
           I2_low_value < -MAX_CURRENT_OPERATION;
}

/**
 * @brief Accumulates squared currents and refreshes the RMS values once per grid period.
 */
void update_rms()
{
    I1_sum_sq += I1_low_value * I1_low_value;
    I2_sum_sq += I2_low_value * I2_low_value;
    rms_sample_count++;

    if (rms_sample_count >= RMS_WINDOW_SAMPLES) {
        I1_rms = sqrtf(I1_sum_sq / static_cast<float32_t>(RMS_WINDOW_SAMPLES));
        I2_rms = sqrtf(I2_sum_sq / static_cast<float32_t>(RMS_WINDOW_SAMPLES));
        I1_sum_sq = 0.0F;
        I2_sum_sq = 0.0F;
        rms_sample_count = 0;
    }
}

/**
 * @brief Tracks the peak current over a grid period and derives RMS as peak/sqrt(2).
 *
 * Valid only if I1_low_value/I2_low_value are close to pure sinusoids: no sqrt() is
 * used, but a single noise spike or harmonic distortion biases the result directly.
 */
void update_rms_peak()
{
    float32_t abs_I1 = fabsf(I1_low_value);
    float32_t abs_I2 = fabsf(I2_low_value);
    if (abs_I1 > I1_peak_running) I1_peak_running = abs_I1;
    if (abs_I2 > I2_peak_running) I2_peak_running = abs_I2;
    peak_sample_count++;

    if (peak_sample_count >= RMS_WINDOW_SAMPLES) {
        I1_rms_peak = I1_peak_running / SQRT2;
        I2_rms_peak = I2_peak_running / SQRT2;
        I1_peak_running = 0.0F;
        I2_peak_running = 0.0F;
        peak_sample_count = 0;
    }
}

/**
 * @brief Tracks an EMA/IIR estimate of the mean-square current and derives RMS continuously.
 *
 * Same discretization as LowPassFirstOrderFilter (see filters.cpp), applied to I^2:
 * meanSq settles toward the true mean square with time constant RMS_EMA_TAU. sqrt() is
 * taken every call, so this trades a smooth, continuously updated output for far more
 * sqrtf() calls than update_rms()/update_rms_peak() (once per cycle vs. once per window).
 */
void update_rms_ema()
{
    I1_meanSq_ema += RMS_EMA_ALPHA * (I1_low_value * I1_low_value - I1_meanSq_ema);
    I2_meanSq_ema += RMS_EMA_ALPHA * (I2_low_value * I2_low_value - I2_meanSq_ema);
    I1_rms_ema = sqrtf(I1_meanSq_ema);
    I2_rms_ema = sqrtf(I2_meanSq_ema);
}

/**
 * @brief Tracks V_high min/max/mean and refreshes the bus voltage ripple once per grid period.
 */
void update_voltage_ripple()
{
    if (V_high_value < Vhigh_min_running) Vhigh_min_running = V_high_value;
    if (V_high_value > Vhigh_max_running) Vhigh_max_running = V_high_value;
    Vhigh_sum += V_high_value;
    vripple_sample_count++;

    if (vripple_sample_count >= RMS_WINDOW_SAMPLES) {
        Vhigh_mean = Vhigh_sum / static_cast<float32_t>(RMS_WINDOW_SAMPLES);
        Vhigh_ripple_pp = Vhigh_max_running - Vhigh_min_running;
        Vhigh_ripple_pct = (Vhigh_mean != 0.0F) ? (Vhigh_ripple_pp / Vhigh_mean) * 100.0F : 0.0F;

        Vhigh_sum = 0.0F;
        Vhigh_min_running = FLT_MAX;
        Vhigh_max_running = -FLT_MAX;
        vripple_sample_count = 0;
    }
}

/*----------------------- END OF SUPPORT FUNCTIONS ------------------------- */

/*----------------------- MAIN APPLICATION CODE ---------------------------- */

/**
 * @brief Configures hardware, scope capture, and OwnTech task scheduling.
 */
void setup_routine()
{
    spin.pwm.initFixedFrequency(50000);
    shield.power.setDeadTime(LEG1, 20, 20);
    shield.power.setDeadTime(LEG2, 20, 20);
    shield.sensors.enableDefaultTwistSensors();
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);

    setup_scope();

    uint32_t app_task_number = task.createBackground(loop_application_task);
    // uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    task.startBackground(app_task_number);
    // task.startBackground(com_task_number);
    task.startCritical();
}

/**
 * @brief Handles serial commands for mode changes, tuning, and scope capture.
 */
void loop_communication_task()
{
    while (1) {
        received_serial_char = console_getchar();
        switch (received_serial_char) {
        case 'h':
            printk(" ________________________________________\n");
            printk("|     open-loop sine PWM                 |\n");
            printk("|     i : idle                           |\n");
            printk("|     p : power                          |\n");
            printk("|     u/j : phi +/- 0.01 rad             |\n");
            printk("|     o/l : phi_ACsource +/- 0.01 rad    |\n");
            printk("|     d/c : Mp +/- 1 %                   |\n");
            printk("|     f/v : Mp_ACsource +/- 1 %          |\n");
            printk("|     r : retrieve scope data            |\n");
            printk("|     t : trigger scope data             |\n");
            printk("|________________________________________|\n\n");
            break;
        case 'i':
            mode_asked = IDLEMODE;
            break;
        case 'p':
            if (!is_downloading) {
                scope.start();
                mode_asked = POWERMODE;
                critical_task_counter = 0;
            }
            break;
        case 'u':
            adjust_phase_converter(0.01F);
            break;
        case 'j':
            adjust_phase_converter(-0.01F);
            break;
        case 'o':
            adjust_phase_ACsource(0.01F);
            break;
        case 'l':
            adjust_phase_ACsource(-0.01F);
            break;
        case 'd':
            adjust_amplitude_converter(0.01F);
            break;
        case 'c':
            adjust_amplitude_converter(-0.01F);
            break;
        case 'f':
            adjust_amplitude_ACsource(0.01F);
            break;
        case 'v':
            adjust_amplitude_ACsource(-0.01F);
            break;
        case 'r':
            is_downloading = true;
            trigger = false;
            break;
        case 't':
            trigger = true;
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Runs the low-rate state machine and status reporting.
 */
void loop_application_task()
{
    spin.led.toggle();

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
    if (meas_data != NO_VALUE) temp_1_value = meas_data;

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);
    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
    if (meas_data != NO_VALUE) temp_2_value = meas_data;

    switch (mode) {
    case IDLEMODE:
        if (mode_asked == POWERMODE) {
            mode = POWERMODE;
        }
        break;
    case POWERMODE:
        if (mode_asked == IDLEMODE) {
            mode = IDLEMODE;
        }
        break;
    case ERRORMODE:
        break;
    default:
        mode = ERRORMODE;
        break;
    }

    if (mode_asked == IDLEMODE) {
        mode = IDLEMODE;
    }

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    } else {
        /*
        printk("state %d:Vdc %.2f:Vgrid %.2f:Vlocal %.2f:Mp %.2f:Mp_src %.2f:phi_m %.2f:phi_src %.2f"
                ":d1 %.3f:d2 %.3f\n",
               mode,
            //    "state %d:Vdc %.2f:Vgrid %.2f:Vlocal %.2f:amp %.2f:d1 %.3f:d2 %.3f"
            //    ":I1rms %.3f:I2rms %.3f:I1rmsPk %.3f:I2rmsPk %.3f:I1rmsEma %.3f:I2rmsEma %.3f"
            //    ":Vpp %.3f:Vpp%% %.2f\n",
            //    mode,
               static_cast<double>(V_high_filt),
               static_cast<double>(Vgrid_meas),
               static_cast<double>(local_vgrid),
               static_cast<double>(Mp),
               static_cast<double>(Mp_AC_source),
               static_cast<double>(phi_m),
               static_cast<double>(phi_AC_source),
               static_cast<double>(duty_cycle_1),
               static_cast<double>(duty_cycle_2));
            //    static_cast<double>(I1_rms),
            //    static_cast<double>(I2_rms),
            //    static_cast<double>(I1_rms_peak),
            //    static_cast<double>(I2_rms_peak),
            //    static_cast<double>(I1_rms_ema),
            //    static_cast<double>(I2_rms_ema),
            //    static_cast<double>(Vhigh_ripple_pp),
            //    static_cast<double>(Vhigh_ripple_pct));
            */
    }

    /* blink_period_s is writable over the ThingSet shell (Config/wBlinkPeriod_s) */
    task.suspendBackgroundMs((uint32_t)(blink_period_s * 1000.0f));
}

/**
 * @brief Runs the 10 kHz control loop, protection checks, and PWM updates.
 */
void loop_critical_task()
{
    critical_task_counter++;
    read_measurements();
    // update_rms();
    // update_rms_peak();
    // update_rms_ema();
    update_voltage_ripple();
    update_teaching_sine();

    // dac_value = delta_duty_cycle * 4000;
    spin.dac.setConstValue(2, 1, dac_value);

    if (overcurrent_detected()) {
        mode = ERRORMODE;
    }

    if (mode == POWERMODE) {
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();

        if (critical_task_counter > 20 && overcurrent_detected()) {
            mode = ERRORMODE;
        }
        if (critical_task_counter > 1000 && overcurrent_detected_operation()) {
            mode = ERRORMODE;
            trigger = true;
        }

    } else {
        stop_pwm_outputs();
        // spin.led.turnOff();
    }

    state_mode_scope = static_cast<float32_t>(mode);
    scope.acquire();
}

/**
 * @brief Application entry point.
 */
int main(void)
{
    setup_routine();
    return 0;
}