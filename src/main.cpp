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
 * @brief  This file is the main application code for the grid-forming
 *         inverter example. It implements a single-phase grid-forming control
 *         strategy that generates its own voltage reference and regulates the
 *         output voltage without requiring external grid synchronization.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

 /*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

/* From control library */
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"
#include "singlePhaseInverter.h"

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
/* Moves a value toward its reference with a fixed slew rate */
float32_t rate_limiter(
    float32_t reference, float32_t value, float32_t rate);
/* Returns the DC bus voltage, or a fallback before sensing is valid */
float32_t control_bus_voltage();
/* Clamps one duty-cycle command to the allowed modulation range */
float32_t clamp_duty(float32_t duty);
/* Starts both PWM legs once and records the output state */
void start_pwm_outputs();
/* Stops both PWM legs once and records the output state */
void stop_pwm_outputs();
/* Applies the same clamped duty cycle to both H-bridge legs */
void apply_common_duty(float32_t duty);
/* Applies clamped complementary duty cycles to the H-bridge legs */
void apply_complementary_duty(float32_t duty);
/* Advances the local oscillator and derives voltage/current inputs */
void update_teaching_sine();
/* Copies the inverter controller diagnostics into scope variables */
void refresh_inverter_data();
/* Streams a completed ScopeMimicry capture over the serial console */
void dump_scope_datas(ScopeMimicry &scope_to_dump);
/* Adjusts the forming-mode voltage reference and teaching amplitude */
void adjust_voltage_reference(float32_t step);
/* Registers scope channels and starts capture for forming example */
void setup_scope();
/* Reads sensor values and derives grid voltage/current measurements */
void read_measurements();
/* Checks both measured currents against the protection threshold */
bool overcurrent_detected();

/*--------------USER VARIABLES DECLARATIONS------------------- */

enum ConverterState : uint8_t /* Holds the current state of the inverter */
{
    IDLEMODE = 0,   /* Idle mode: stops the converter power */
    POWERMODE = 1,  /* Power mode: generates voltage for the load */
    ERRORMODE = 3,  /* Error mode: indicates an error condition */
    STARTUPMODE = 4 /* Startup mode: ramps the duty cycle to neutral */
};

/* Control task period in microseconds */
static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
/* Control task period in seconds */
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
/* Fallback DC bus voltage in volts before sensing is valid */
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
/* DC bus voltage threshold required to leave idle mode */
static constexpr float32_t UDC_STARTUP = 20.0F;
/* Neutral duty-cycle target used while precharging the H-bridge */
static constexpr float32_t STARTUP_DUTY_TARGET = 0.5F;
/* Grid frequency in hertz */
static constexpr float32_t F0 = 50.0F;
/* Grid pulsation in radians per second */
static constexpr float32_t W0 = 2.0F * PI * F0;
/* Load resistance in ohms */
static constexpr float32_t LOAD_RESISTANCE = 10.0F;
/* Maximum current for overcurrent protection in amps */
static constexpr float32_t MAX_CURRENT = 8.0F;
/* Size of the scope buffer for data recording */
static constexpr uint32_t SCOPE_BUFFER_SIZE = 1024;
/* Number of channels recorded in the scope for diagnostics */
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 17;

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
static float32_t V1_low_value;
/* [V] Low-side filtered measurement of voltage 2 */
static float32_t V2_low_value;
/* [A] Low-side filtered measurement of current 1 */
static float32_t I1_low_value;
/* [A] Low-side filtered measurement of current 2 */
static float32_t I2_low_value;
/* [V] High-side raw measurement of voltage */
static float32_t V_high;
/* [V] Further filtered high-side voltage for control */
static float32_t V_high_filt;
/* [V] Measured grid voltage from the difference of low-side measurements */
static float32_t Vgrid_meas;
/* [A] Measured grid current from the low-side current measurement */
static float32_t Igrid_meas;
/* [V] Temporary variable for storing sensor measurements */
static float32_t meas_data;

/* [V] Amplitude of the local teaching sine wave */
static float32_t local_voltage_amplitude = 12.0F;
/* [rad] Phase angle estimated by the inverter controller */
static float32_t inverter_theta;
/* [V] Instantaneous local grid voltage from the teaching sine */
static float32_t local_vgrid;
/* [A] Instantaneous local grid current from the teaching sine */
static float32_t local_igrid;

/* [V] DQ-axis voltage in the synchronous reference frame */
static dqo_t Vdq;
/* [V] DQ-axis voltage output from the inverter after modulation */
static dqo_t Vdq_output;
/* [V] DQ-axis voltage reference for the forming controller */
static dqo_t Vdq_ref;
/* [V] maximum DQ-axis voltage reference for the forming controller */
static dqo_t Vdq_ref_max = {30.0F, 30.0F, 0.0F};
/* [A] DQ-axis current in the synchronous reference frame */
static dqo_t Idq;
/* [A] DQ-axis current reference delta for control adjustments */
static dqo_t Idq_ref_delta;

/* [No unit] Duty cycle computed by the forming controller */
static float32_t delta_duty_cycle;
/* [No unit] Duty cycle for leg 1 of the H-bridge */
static float32_t duty_cycle_1 = 0.5F;
/* [No unit] Duty cycle for leg 2 of the H-bridge */
static float32_t duty_cycle_2 = 0.5F;
/* [rad/s] Estimated grid frequency from the inverter controller */
static float32_t omega = W0;
/* [deg] Current phase shift applied to leg 2 */
static float32_t phase_shift_deg = 0.0F;
/* [No unit] Scope variable for the current operating mode */
static float32_t state_mode_scope;
/* Counter for the number of critical task iterations */
static uint32_t critical_task_counter;

/* Instance of the single-phase inverter control class (forming mode) */
static singlePhaseInverter inverter;
/* First-order low-pass filter for the high-side voltage measurement */
static LowPassFirstOrderFilter vHighFilter(TS, 0.1F);
/* ScopeMimicry instance for recording control variables and diagnostics */
static ScopeMimicry scope(SCOPE_BUFFER_SIZE, SCOPE_CHANNEL_COUNT);
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
 * @brief Moves a value toward its reference with a fixed slew rate.
 */
float32_t rate_limiter(float32_t reference, float32_t value, float32_t rate)
{
    const float32_t step = TS * rate;
    const float32_t error = reference - value;

    if (error > step) {
        return value + step;
    }
    if (error < -step) {
        return value - step;
    }
    return reference;
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
 * @brief Clamps one duty-cycle command to the allowed modulation range.
 */
float32_t clamp_duty(float32_t duty)
{
    return saturate(duty, DUTY_MIN, DUTY_MAX);
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
 * @brief Applies the same clamped duty cycle to both H-bridge legs.
 */
void apply_common_duty(float32_t duty)
{
    duty_cycle_1 = clamp_duty(duty);
    duty_cycle_2 = clamp_duty(duty);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

/**
 * @brief Applies clamped complementary duty cycles to the H-bridge legs.
 */
void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = clamp_duty(duty);
    duty_cycle_2 = clamp_duty(1.0F - duty);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

/**
 * @brief Derives voltage/current inputs aligned with the inverter's own phase angle.
 */
void update_teaching_sine()
{
    local_vgrid = local_voltage_amplitude * ot_sin(inverter_theta);
    local_igrid = local_vgrid / LOAD_RESISTANCE;
}

/**
 * @brief Copies the inverter controller diagnostics into scope variables.
 */
void refresh_inverter_data()
{
    Vdq = inverter.getVdq();
    Vdq_output = inverter.getVdqOut();
    Idq = inverter.getIdq();
    Idq_ref_delta = inverter.getIdqRefDelta();
    inverter_theta = inverter.getTheta();
    omega = inverter.getw();
    phase_shift_deg = inverter.getPhaseShiftDeg();
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
 * @brief Adjusts the forming-mode voltage reference and matching teaching sine amplitude.
 */
void adjust_voltage_reference(float32_t step)
{
    Vdq_ref.d = saturate(Vdq_ref.d + step, 0.0F, Vdq_ref_max.d);
    local_voltage_amplitude = Vdq_ref.d;
}

/**
 * @brief Registers scope channels and starts capture for the grid-forming example.
 */
void setup_scope()
{
    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    scope.connectChannel(Igrid_meas, "Igrid");
    scope.connectChannel(local_vgrid, "local_vgrid");
    scope.connectChannel(local_igrid, "local_igrid");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
    scope.connectChannel(V_high_filt, "Vdc");
    scope.connectChannel(Vdq_ref.d, "Vd_ref");
    scope.connectChannel(Vdq.d, "Vd_in");
    scope.connectChannel(Vdq.q, "Vq_in");
    scope.connectChannel(Idq.d, "Id_in");
    scope.connectChannel(Vdq_output.d, "Vd_out");
    scope.connectChannel(omega, "omega");
    scope.connectChannel(phase_shift_deg, "phase_shift");
    scope.connectChannel(state_mode_scope, "state");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
}

/**
 * @brief Reads the latest sensor values and derives grid voltage/current measurements.
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
    if (meas_data != NO_VALUE) V_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);

    Vgrid_meas = V1_low_value - V2_low_value;
    Igrid_meas = I1_low_value;
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

/*----------------------- END OF SUPPORT FUNCTIONS ------------------------- */

/*----------------------- MAIN APPLICATION CODE ---------------------------- */

/**
 * @brief Configures hardware, scope capture, inverter control, and task scheduling.
 */
void setup_routine()
{
    spin.pwm.initFixedFrequency(200000);
    shield.power.setDeadTime(LEG1, 100, 100);
    shield.power.setDeadTime(LEG2, 100, 100);
    shield.sensors.enableDefaultTwistSensors();
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);

    Vdq_ref.d = local_voltage_amplitude;
    Vdq_ref.q = 0.0F;
    setup_scope();
    inverter.init(FORMING, DC_BUS_FALLBACK, local_voltage_amplitude, W0, TS);

    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/**
 * @brief Handles serial commands for mode changes, voltage tuning, and scope capture.
 */
void loop_communication_task()
{
    while (1) {
        received_serial_char = console_getchar();
        switch (received_serial_char) {
        case 'h':
            printk(" ________________________________________\n");
            printk("|     grid-forming local sine            |\n");
            printk("|     i : idle                           |\n");
            printk("|     p : power                          |\n");
            printk("|     u/j : Vd reference +/- 1 V         |\n");
            printk("|     d/c : Vd reference +/- 5 V         |\n");
            printk("|     f/g : leg 2 phase shift +/- 1 deg  |\n");
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
            }
            break;
        case 'u':
            adjust_voltage_reference(1.0F);
            break;
        case 'j':
            adjust_voltage_reference(-1.0F);
            break;
        case 'd':
            adjust_voltage_reference(5.0F);
            break;
        case 'c':
            adjust_voltage_reference(-5.0F);
            break;
        case 'f':
            inverter.stepPhaseShiftUp();
            break;
        case 'g':
            inverter.stepPhaseShiftDown();
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
    switch (mode) {
    case IDLEMODE:
        if (mode_asked == POWERMODE && V_high_filt >= UDC_STARTUP) {
            mode = STARTUPMODE;
        }
        spin.led.turnOn();
        break;
    case STARTUPMODE:
        if (delta_duty_cycle >= STARTUP_DUTY_TARGET) {
            mode = POWERMODE;
            printk("Entered POWERMODE\n");
        }
        break;
    case POWERMODE:
        if (mode_asked == IDLEMODE) {
            mode = IDLEMODE;
        }
        spin.led.toggle();
        break;
    case ERRORMODE:
        break;
    default:
        mode = ERRORMODE;
        break;
    }

    if (mode_asked == IDLEMODE) {
        mode = IDLEMODE;
        delta_duty_cycle = 0.0F; /* Reset delta duty cycle */
    }

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    } else {
        printk("state %d:Vdc %.2f:Vlocal %.2f:Vdref %.2f:Vd %.2f:Id %.2f:d1 %.3f:d2 %.3f: delta_duty_cycle %.6f\n",
               mode,
               static_cast<double>(V_high_filt),
               static_cast<double>(local_vgrid),
               static_cast<double>(Vdq_ref.d),
               static_cast<double>(Vdq.d),
               static_cast<double>(Idq.d),
               static_cast<double>(duty_cycle_1),
               static_cast<double>(duty_cycle_2),
               static_cast<double>(delta_duty_cycle));
    }

    task.suspendBackgroundMs(100);
}

/**
 * @brief Runs the 10 kHz forming control loop, protection checks, and PWM updates.
 */
void loop_critical_task()
{
    critical_task_counter++;
    read_measurements();
    update_teaching_sine();

    if (overcurrent_detected()) {
        mode = ERRORMODE;
    }

    if (mode == IDLEMODE || mode == ERRORMODE) {
        stop_pwm_outputs();
        inverter.setPowerOn(false);
    } else if (mode == STARTUPMODE) {
        delta_duty_cycle = rate_limiter(
            STARTUP_DUTY_TARGET, delta_duty_cycle, 50.0F);

        apply_common_duty(delta_duty_cycle);
        
        start_pwm_outputs();

    } else if (mode == POWERMODE) {
        inverter.setPowerOn(true);
        inverter.setVBus(control_bus_voltage());
        inverter.setVdqRef(Vdq_ref);
        delta_duty_cycle = inverter.calculateDuty(Vgrid_meas, Igrid_meas);
        duty_cycle_1 = clamp_duty(delta_duty_cycle);
        duty_cycle_2 = clamp_duty(inverter.getDutyLeg2());
        shield.power.setDutyCycle(LEG1, duty_cycle_1);
        shield.power.setDutyCycle(LEG2, duty_cycle_2);
        start_pwm_outputs();
    }

    refresh_inverter_data();
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
