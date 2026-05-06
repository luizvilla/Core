/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
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

#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"
#include "singlePhaseInverter.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F

enum ConverterState : uint8_t
{
    IDLEMODE = 0,
    POWERMODE = 1,
    ERRORMODE = 3,
    STARTUPMODE = 4
};

enum TeachingMode : uint8_t
{
    OPEN_LOOP = 1,
    GRID_FORMING_LOCAL_SINE = 2,
    GRID_FOLLOWING_LOCAL_PLL = 3,
    GRID_FOLLOWING_MEASURED_PLL = 4
};

static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
static constexpr float32_t UDC_STARTUP = 0.0F;
static constexpr float32_t F0 = 50.0F;
static constexpr float32_t W0 = 2.0F * PI * F0;
static constexpr float32_t SYNC_POWER_TOLERANCE = 0.01F * W0;
static constexpr float32_t LOAD_RESISTANCE = 20.0F;
static constexpr float32_t MAX_CURRENT = 8.0F;
static constexpr uint32_t SCOPE_BUFFER_SIZE = 1024;
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 21;

static bool pwm_enable = false;
static bool is_downloading = false;
static bool trigger = false;
static bool is_net_synchronized = false;

static uint8_t received_serial_char;
static uint8_t mode = IDLEMODE;
static uint8_t mode_asked = IDLEMODE;
static TeachingMode teaching_mode = GRID_FORMING_LOCAL_SINE;
static inverter_mode local_mode = FORMING;

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t V_high;
static float32_t V_high_filt;
static float32_t Vgrid_meas;
static float32_t Igrid_meas;
static float32_t meas_data;

static float32_t local_voltage_amplitude = 20.0F;
static float32_t teaching_theta;
static float32_t inverter_theta;
static float32_t sine;
static float32_t local_vgrid;
static float32_t local_igrid;

static dqo_t Vdq;
static dqo_t Vdq_output;
static dqo_t Vdq_ref;
static dqo_t Idq;
static dqo_t Idq_ref;
static dqo_t Idq_ref_delta;

static float32_t delta_duty_cycle;
static float32_t duty_cycle_1 = 0.5F;
static float32_t duty_cycle_2 = 0.5F;
static float32_t omega = W0;
static float32_t teaching_mode_scope = GRID_FORMING_LOCAL_SINE;
static float32_t state_mode_scope;
static float32_t sync_scope;
static uint32_t critical_task_counter;
static uint32_t desync_counter;

static singlePhaseInverter inverter;
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

float32_t sign(float32_t value, float32_t tolerance = 1.0e-3F)
{
    if (value > tolerance) {
        return 1.0F;
    }
    if (value < -tolerance) {
        return -1.0F;
    }
    return 0.0F;
}

float32_t rate_limiter(float32_t reference, float32_t value, float32_t rate)
{
    value += TS * rate * sign(reference - value);
    return value;
}

float32_t clamp_duty(float32_t duty)
{
    return saturate(duty, DUTY_MIN, DUTY_MAX);
}

float32_t control_bus_voltage()
{
    if (V_high_filt > 1.0F) {
        return V_high_filt;
    }
    return DC_BUS_FALLBACK;
}

void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = clamp_duty(duty);
    duty_cycle_2 = clamp_duty(1.0F - duty);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

void apply_common_duty(float32_t duty)
{
    duty_cycle_1 = clamp_duty(duty);
    duty_cycle_2 = clamp_duty(duty);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
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

bool is_following_mode()
{
    return teaching_mode == GRID_FOLLOWING_LOCAL_PLL ||
           teaching_mode == GRID_FOLLOWING_MEASURED_PLL;
}

inverter_mode inverter_mode_for_teaching_mode(TeachingMode requested_mode)
{
    if (requested_mode == GRID_FOLLOWING_LOCAL_PLL ||
        requested_mode == GRID_FOLLOWING_MEASURED_PLL) {
        return FOLLOWING;
    }
    return FORMING;
}

void update_teaching_sine()
{
    teaching_theta = ot_modulo_2pi(teaching_theta + W0 * TS);
    sine = ot_sin(teaching_theta);
    local_vgrid = local_voltage_amplitude * sine;
    local_igrid = local_vgrid / LOAD_RESISTANCE;
}

float32_t following_vgrid_input()
{
    if (teaching_mode == GRID_FOLLOWING_LOCAL_PLL) {
        return local_vgrid;
    }
    return Vgrid_meas;
}

float32_t following_igrid_input()
{
    if (teaching_mode == GRID_FOLLOWING_LOCAL_PLL) {
        return local_igrid;
    }
    return Igrid_meas;
}

bool following_frequency_in_range()
{
    return omega <= W0 + SYNC_POWER_TOLERANCE &&
           omega >= W0 - SYNC_POWER_TOLERANCE;
}

void refresh_inverter_data()
{
    Vdq = inverter.getVdq();
    Vdq_output = inverter.getVdqOut();
    Idq = inverter.getIdq();
    Idq_ref_delta = inverter.getIdqRefDelta();
    inverter_theta = inverter.getTheta();
    omega = inverter.getw();
    sync_scope = is_net_synchronized ? 1.0F : 0.0F;
}

void configure_teaching_mode(TeachingMode requested_mode)
{
    teaching_mode = requested_mode;
    teaching_mode_scope = static_cast<float32_t>(requested_mode);
    local_mode = inverter_mode_for_teaching_mode(teaching_mode);
    mode = IDLEMODE;
    mode_asked = IDLEMODE;
    is_net_synchronized = false;
    desync_counter = 0;
    delta_duty_cycle = 0.0F;
    duty_cycle_1 = 0.5F;
    duty_cycle_2 = 0.5F;
    inverter.init(local_mode, DC_BUS_FALLBACK, local_voltage_amplitude, W0, TS);
    inverter.setPowerOn(false);
    stop_pwm_outputs();
}

void dump_scope_datas(ScopeMimicry &scope_to_dump)
{
    scope_to_dump.reset_dump();
    printk("begin record\n");
    while(scope_to_dump.get_dump_state() != finished) {
        printk("%s", scope_to_dump.dump_datas());
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

void handle_following_desync()
{
    if (is_net_synchronized) {
        desync_counter = 0;
        return;
    }

    desync_counter++;
    if (desync_counter > 200) {
        desync_counter = 0;
        mode_asked = IDLEMODE;
        mode = IDLEMODE;
        inverter.setPowerOn(false);
        stop_pwm_outputs();
        printk("System no longer synchronized\n");
    }
}

void adjust_reference(float32_t forming_step, float32_t following_step)
{
    if (local_mode == FORMING) {
        Vdq_ref.d = saturate(Vdq_ref.d + forming_step, 0.0F, 30.0F);
        local_voltage_amplitude = Vdq_ref.d;
    } else {
        Idq_ref.d = saturate(Idq_ref.d + following_step, -0.1F, 8.0F);
    }
}

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
    scope.connectChannel(Vdq.d, "Vd_in");
    scope.connectChannel(Vdq.q, "Vq_in");
    scope.connectChannel(Idq.d, "Id_in");
    scope.connectChannel(Idq.q, "Iq_in");
    scope.connectChannel(Vdq_output.d, "Vd_out");
    scope.connectChannel(Vdq_output.q, "Vq_out");
    scope.connectChannel(inverter_theta, "theta");
    scope.connectChannel(omega, "omega");
    scope.connectChannel(sync_scope, "sync");
    scope.connectChannel(teaching_mode_scope, "teaching_mode");
    scope.connectChannel(state_mode_scope, "state");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
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

    Vdq_ref.d = local_voltage_amplitude;
    Vdq_ref.q = 0.0F;
    Idq_ref.d = 0.0F;
    Idq_ref.q = 0.0F;
    Idq_ref_delta.d = 0.0F;
    Idq_ref_delta.q = 0.0F;

    setup_scope();
    inverter.init(local_mode, DC_BUS_FALLBACK, local_voltage_amplitude, W0, TS);

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
            printk("|     single-phase inverter teaching     |\n");
            printk("|     i : idle                           |\n");
            printk("|     p : power                          |\n");
            printk("|     1 : open-loop sine PWM             |\n");
            printk("|     2 : forming, local sine            |\n");
            printk("|     3 : following PLL, local sine      |\n");
            printk("|     4 : following PLL, measurements    |\n");
            printk("|     u/j : reference +/- small step     |\n");
            printk("|     d/c : reference +/- large step     |\n");
            printk("|     r : retrieve scope data            |\n");
            printk("|     t : trigger scope data             |\n");
            printk("|________________________________________|\n\n");
            break;
        case 'i':
            printk("idle mode\n");
            mode_asked = IDLEMODE;
            break;
        case 'p':
            if (!is_downloading) {
                printk("power mode\n");
                scope.start();
                mode_asked = POWERMODE;
            }
            break;
        case '1':
            configure_teaching_mode(OPEN_LOOP);
            printk("open-loop sine PWM\n");
            break;
        case '2':
            configure_teaching_mode(GRID_FORMING_LOCAL_SINE);
            printk("grid forming with local sine\n");
            break;
        case '3':
            configure_teaching_mode(GRID_FOLLOWING_LOCAL_PLL);
            printk("grid following PLL with local sine\n");
            break;
        case '4':
            configure_teaching_mode(GRID_FOLLOWING_MEASURED_PLL);
            printk("grid following PLL with measurements\n");
            break;
        case 'u':
            adjust_reference(1.0F, 0.1F);
            break;
        case 'j':
            adjust_reference(-1.0F, -0.1F);
            break;
        case 'd':
            adjust_reference(5.0F, 1.0F);
            break;
        case 'c':
            adjust_reference(-5.0F, -1.0F);
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
        if (mode_asked == POWERMODE && V_high_filt >= UDC_STARTUP) {
            mode = STARTUPMODE;
        }
        spin.led.turnOn();
        break;
    case STARTUPMODE:
        if (!is_following_mode() && delta_duty_cycle >= 0.5F) {
            mode = POWERMODE;
        } else if (is_following_mode() && is_net_synchronized) {
            mode = POWERMODE;
        }
        break;
    case POWERMODE:
        if (mode_asked == IDLEMODE) {
            mode = IDLEMODE;
        }
        if (is_net_synchronized) {
            spin.led.toggle();
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
        printk("state %d:teaching %d:sync %d:Vdc %.2f:Vgrid %.2f:Vlocal %.2f:Vdref %.2f:Idref %.2f:omega %.0f\n",
               mode,
               teaching_mode,
               is_net_synchronized ? 1 : 0,
               static_cast<double>(V_high_filt),
               static_cast<double>(Vgrid_meas),
               static_cast<double>(local_vgrid),
               static_cast<double>(Vdq_ref.d),
               static_cast<double>(Idq_ref.d),
               static_cast<double>(omega));
    }

    task.suspendBackgroundMs(100);
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
    Igrid_meas = I1_low_value;
}

bool overcurrent_detected()
{
    return I1_low_value > MAX_CURRENT ||
           I1_low_value < -MAX_CURRENT ||
           I2_low_value > MAX_CURRENT ||
           I2_low_value < -MAX_CURRENT;
}

void run_startup_mode()
{
    if (is_following_mode()) {
        inverter.setVBus(control_bus_voltage());
        inverter.setPowerOn(false);
        delta_duty_cycle = inverter.calculateDuty(following_vgrid_input(), following_igrid_input());
        refresh_inverter_data();
        is_net_synchronized = inverter.getSync() && following_frequency_in_range();
        sync_scope = is_net_synchronized ? 1.0F : 0.0F;
        return;
    }

    delta_duty_cycle = rate_limiter(0.5F, delta_duty_cycle, 50.0F);
    if (delta_duty_cycle > 0.5F) {
        delta_duty_cycle = 0.5F;
    }
    apply_common_duty(delta_duty_cycle);
    start_pwm_outputs();
}

void run_power_mode()
{
    if (teaching_mode == OPEN_LOOP) {
        delta_duty_cycle = 0.5F + local_vgrid / (2.0F * control_bus_voltage());
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();
        return;
    }

    inverter.setVBus(control_bus_voltage());

    if (teaching_mode == GRID_FORMING_LOCAL_SINE) {
        inverter.setVdqRef(Vdq_ref);
        delta_duty_cycle = inverter.calculateDuty(local_vgrid, local_igrid);
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();
        return;
    }

    inverter.setIdqRef(Idq_ref);
    inverter.setPowerOn(true);
    delta_duty_cycle = inverter.calculateDuty(following_vgrid_input(), following_igrid_input());
    refresh_inverter_data();
    is_net_synchronized = inverter.getSync() && following_frequency_in_range();
    sync_scope = is_net_synchronized ? 1.0F : 0.0F;
    handle_following_desync();

    if (is_net_synchronized) {
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();
    } else {
        inverter.setPowerOn(false);
    }
}

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
        spin.led.turnOff();
    } else if (mode == STARTUPMODE) {
        run_startup_mode();
    } else if (mode == POWERMODE) {
        run_power_mode();
    }

    refresh_inverter_data();
    state_mode_scope = static_cast<float32_t>(mode);

    if (critical_task_counter % 1 == 0) {
        scope.acquire();
    }
}

int main(void)
{
    setup_routine();
    return 0;
}
