/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F

enum ConverterState : uint8_t
{
    IDLEMODE = 0,
    POWERMODE = 1,
    ERRORMODE = 3
};

static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
static constexpr float32_t F0 = 50.0F;
static constexpr float32_t W0 = 2.0F * PI * F0;
static constexpr float32_t MAX_CURRENT = 8.0F;
static constexpr uint32_t SCOPE_BUFFER_SIZE = 1024;
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 10;

static bool pwm_enable = false;
static bool is_downloading = false;
static bool trigger = false;

static uint8_t received_serial_char;
static uint8_t mode = IDLEMODE;
static uint8_t mode_asked = IDLEMODE;

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t V_high;
static float32_t V_high_filt;
static float32_t Vgrid_meas;
static float32_t meas_data;

static float32_t local_voltage_amplitude = 20.0F;
static float32_t teaching_theta;
static float32_t sine;
static float32_t local_vgrid;
static float32_t delta_duty_cycle = 0.5F;
static float32_t duty_cycle_1 = 0.5F;
static float32_t duty_cycle_2 = 0.5F;
static float32_t state_mode_scope;
static uint32_t critical_task_counter;

static LowPassFirstOrderFilter vHighFilter(TS, 0.1F);
static ScopeMimicry scope(SCOPE_BUFFER_SIZE, SCOPE_CHANNEL_COUNT);

void setup_routine();
void loop_communication_task();
void loop_application_task();
void loop_critical_task();

bool a_trigger()
{
    return trigger;
}

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

float32_t control_bus_voltage()
{
    if (V_high_filt > 1.0F) {
        return V_high_filt;
    }
    return DC_BUS_FALLBACK;
}

void start_pwm_outputs()
{
    if (!pwm_enable) {
        shield.power.start(ALL);
        pwm_enable = true;
    }
}

void stop_pwm_outputs()
{
    if (pwm_enable) {
        shield.power.stop(ALL);
        pwm_enable = false;
    }
}

void update_teaching_sine()
{
    teaching_theta = ot_modulo_2pi(teaching_theta + W0 * TS);
    sine = ot_sin(teaching_theta);
    local_vgrid = local_voltage_amplitude * sine;
}

void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = saturate(duty, DUTY_MIN, DUTY_MAX);
    duty_cycle_2 = saturate(1.0F - duty, DUTY_MIN, DUTY_MAX);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

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

void adjust_amplitude(float32_t step)
{
    local_voltage_amplitude = saturate(local_voltage_amplitude + step, 0.0F, 30.0F);
}

void setup_scope()
{
    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    scope.connectChannel(V_high_filt, "Vdc");
    scope.connectChannel(local_vgrid, "local_vgrid");
    scope.connectChannel(sine, "sine");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
    scope.connectChannel(state_mode_scope, "state");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
}

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
}

bool overcurrent_detected()
{
    return I1_low_value > MAX_CURRENT ||
           I1_low_value < -MAX_CURRENT ||
           I2_low_value > MAX_CURRENT ||
           I2_low_value < -MAX_CURRENT;
}

void setup_routine()
{
    spin.pwm.initFixedFrequency(50000);
    shield.power.setDeadTime(LEG1, 20, 20);
    shield.power.setDeadTime(LEG2, 20, 20);
    shield.sensors.enableDefaultTwistSensors();
    shield.power.connectCapacitor(LEG1);
    shield.power.connectCapacitor(LEG2);
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);

    setup_scope();

    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

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
            printk("|     u/j : amplitude +/- 1 V            |\n");
            printk("|     d/c : amplitude +/- 5 V            |\n");
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
            adjust_amplitude(1.0F);
            break;
        case 'j':
            adjust_amplitude(-1.0F);
            break;
        case 'd':
            adjust_amplitude(5.0F);
            break;
        case 'c':
            adjust_amplitude(-5.0F);
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

void loop_application_task()
{
    switch (mode) {
    case IDLEMODE:
        if (mode_asked == POWERMODE) {
            mode = POWERMODE;
        }
        spin.led.turnOn();
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
    }

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    } else {
        printk("state %d:Vdc %.2f:Vgrid %.2f:Vlocal %.2f:amp %.2f:d1 %.3f:d2 %.3f\n",
               mode,
               static_cast<double>(V_high_filt),
               static_cast<double>(Vgrid_meas),
               static_cast<double>(local_vgrid),
               static_cast<double>(local_voltage_amplitude),
               static_cast<double>(duty_cycle_1),
               static_cast<double>(duty_cycle_2));
    }

    task.suspendBackgroundMs(100);
}

void loop_critical_task()
{
    critical_task_counter++;
    read_measurements();
    update_teaching_sine();

    if (overcurrent_detected()) {
        mode = ERRORMODE;
    }

    if (mode == POWERMODE) {
        delta_duty_cycle = 0.5F + local_vgrid / (2.0F * control_bus_voltage());
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();
    } else {
        stop_pwm_outputs();
        spin.led.turnOff();
    }

    state_mode_scope = static_cast<float32_t>(mode);
    scope.acquire();
}

int main(void)
{
    setup_routine();
    return 0;
}
