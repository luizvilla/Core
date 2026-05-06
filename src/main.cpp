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

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

// from control library
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"
#include "singlePhaseInverter.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F
#define UDC_STARTUP 0.0F
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine();           /* Setups the hardware and software of the system */

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task
void configure_teaching_mode(uint8_t requested_mode);

//--------------USER VARIABLES DECLARATIONS-------------------
static const uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */
static float32_t V1_low_value; // [V]
static float32_t V2_low_value; // [V]
static float32_t I1_low_value; // [A]
static float32_t I2_low_value; // [A]
static float32_t V_high; // [V]
static float32_t I_high; // [A]
static float32_t V_high_filt; // [V]

static float32_t I1_current_offset = 0.25; // [A] Current offset found experimentally 21/10/2025
static float32_t I2_current_offset = 0.25; // [A]


static float32_t Vgrid_meas; // [V]
static float32_t VN_meas; // [V]
static float32_t Igrid_meas; // [V]


static float meas_data; // temp storage meas value (ctrl task)

static singlePhaseInverter inverter;

static dqo_t power;


static dqo_t Vdq; // Vdq measure (in)
static dqo_t Vdq_output; // Inverter output
static dqo_t Vdq_ref;
static dqo_t Vdq_ref_max;
static dqo_t Vdq_ref_min; 
static float32_t Valpha_in_out;

static dqo_t Idq;
static dqo_t Idq_ref;
static dqo_t Idq_ref_max;
static dqo_t Idq_ref_min;
static dqo_t Idq_ref_delta;




static clarke_t Vab;
static clarke_t Vab_output;
static clarke_t Iab;

static bool is_net_synchronized;
static float32_t omega;

static inverter_mode local_mode = FORMING;
enum TeachingMode
{
    OPEN_LOOP = 1,
    GRID_FORMING_LOCAL_SINE = 2,
    GRID_FOLLOWING_LOCAL_PLL = 3,
    GRID_FOLLOWING_MEASURED_PLL = 4
};

static TeachingMode teaching_mode = GRID_FORMING_LOCAL_SINE;
static float32_t teaching_mode_scope = GRID_FORMING_LOCAL_SINE;

/* duty_cycle*/
static float32_t delta_duty_cycle;// [No unit]
static float32_t duty_cycle_1;// [No unit]
static float32_t duty_cycle_2;// [No unit]
static float32_t duty_cycle_offset;// [No unit]

static float32_t Udc = 20.0F; // dc voltage supply assumed [V]
static const float f0 = 50.0F; // fundamental frequency [Hz]
static const float32_t w0 = 2.0F * PI * f0;   // pulsation [rad/s]
static const float32_t sync_power_tolerance = 0.01*w0;
static const float32_t LOAD_RESISTANCE = 20.0F;
/* Sinewave settings */
static float32_t Vgrid_ref; //[V]
static float32_t Vgrid_amplitude_ref = 20.0F; // [V]
static float32_t Vgrid_amplitude = 20.0F; // [V]
static float angle = 0.F; // [rad]
static float theta = 0.F; // [rad]
static float32_t sine = 0.F; // [rad]
static float32_t local_vgrid; // [V]
static float32_t local_igrid; // [A]

static float32_t Ts = control_task_period * 1.0e-6F;

// comes from "filters.h"
LowPassFirstOrderFilter vHighFilter(Ts, 0.1F);
static uint32_t critical_task_counter;
static uint32_t decimation = 1;
static uint32_t sync_counter=0;
static uint32_t desync_counter=0;
static uint32_t power_counter=0;
static float32_t desync_counter_scope;

// the scope help us to record datas during the critical task
// its a library which must be included in platformio.ini
static ScopeMimicry scope(1024, 22);
static bool is_downloading;
static bool trigger = false;
//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE=1,
    ERRORMODE=3,
    STARTUPMODE=4
};

static uint8_t mode = IDLEMODE;
static uint8_t mode_asked = IDLEMODE;
static float32_t spying_mode = 0;
static const float32_t MAX_CURRENT = 8.0F;

bool a_trigger() {
    return trigger;
}

/**
 * @brief print recorded data of the ScopeMimicry instance to console
 * we use this function in coordination with a miniterm python filter on the host side.
 * `filter_recorded_data.py` to save the data in a file and format them in float.
 *
 * @param scope
 */
void dump_scope_datas(ScopeMimicry &scope)  {
	scope.reset_dump();
    printk("begin record\n");
	while(scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(100);
	}
    printk("end record\n");
}

// UTILS FUNCTIONS FOR CONTROL
float32_t saturate(const float32_t x, float32_t min, float32_t max) {
    if (x > max) {
        return max;
    }
    if (x < min) {
        return min;
    }
    return x;
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
    return Udc;
}

void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = clamp_duty(duty);
    duty_cycle_2 = clamp_duty(1.0F - duty);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

float32_t sign(float32_t x, float32_t tol=1e-3) {
    if (x > tol) {
        return 1.0F;
    }
    if (x < -tol) {
        return -1.0F;
    }
    return 0.0F;
}

float32_t rate_limiter(const float32_t ref, float32_t value, const float32_t rate) {
    value += Ts * rate * sign(ref - value);
    return value;
}

void update_teaching_sine()
{
    theta = ot_modulo_2pi(theta + w0 * Ts);
    sine = ot_sin(theta);
    local_vgrid = Vgrid_amplitude_ref * sine;
    local_igrid = local_vgrid / LOAD_RESISTANCE;
}

inverter_mode inverter_mode_for_teaching_mode(TeachingMode requested_mode)
{
    if (requested_mode == GRID_FOLLOWING_LOCAL_PLL ||
        requested_mode == GRID_FOLLOWING_MEASURED_PLL)
    {
        return FOLLOWING;
    }
    return FORMING;
}

void stop_pwm_outputs()
{
    if (pwm_enable)
    {
        shield.power.stop(ALL);
        pwm_enable = false;
    }
}

void configure_teaching_mode(uint8_t requested_mode)
{
    if (requested_mode < OPEN_LOOP || requested_mode > GRID_FOLLOWING_MEASURED_PLL)
    {
        return;
    }

    teaching_mode = static_cast<TeachingMode>(requested_mode);
    teaching_mode_scope = static_cast<float32_t>(requested_mode);
    local_mode = inverter_mode_for_teaching_mode(teaching_mode);
    mode = IDLEMODE;
    mode_asked = IDLEMODE;
    is_net_synchronized = false;
    sync_counter = 0;
    desync_counter = 0;
    power_counter = 0;
    delta_duty_cycle = 0.0F;
    duty_cycle_1 = 0.5F;
    duty_cycle_2 = 0.5F;
    duty_cycle_offset = 0.5F;
    inverter.init(local_mode, Udc, Vgrid_amplitude_ref, w0, Ts);
    inverter.setPowerOn(false);
    stop_pwm_outputs();
}

float32_t following_vgrid_input()
{
    if (teaching_mode == GRID_FOLLOWING_LOCAL_PLL)
    {
        return local_vgrid;
    }
    return Vgrid_meas;
}

float32_t following_igrid_input()
{
    if (teaching_mode == GRID_FOLLOWING_LOCAL_PLL)
    {
        return local_igrid;
    }
    return Igrid_meas;
}

void handle_following_desync()
{
    if (is_net_synchronized)
    {
        desync_counter = 0;
        return;
    }

    desync_counter++;
    desync_counter_scope = static_cast<float32_t>(desync_counter);
    if (desync_counter > 200)
    {
        desync_counter = 0;
        sync_counter = 0;
        mode_asked = IDLEMODE;
        mode = IDLEMODE;
        inverter.setPowerOn(false);
        printk("System no longer synchronized \n");
    }
}

bool following_frequency_in_range()
{
    return omega <= w0 + sync_power_tolerance && omega >= w0 - sync_power_tolerance;
}

//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your hardware and tasks.
 * In this example, we setup the version of the spin board and a 
 * background task. The critical task is defined but not started.
 * NOTE: It is important to follow the steps and initialize the hardware first 
 * and the tasks second. 
 */
void setup_routine()
{
    // Setup the hardware first
    spin.pwm.initFixedFrequency(50000);
    shield.power.setDeadTime(LEG1,20,20);
    shield.power.setDeadTime(LEG2,20,20);
    shield.sensors.enableDefaultTwistSensors();

    // DISABLE DC LOW CAPACITORS
    shield.power.connectCapacitor(LEG1);
    shield.power.connectCapacitor(LEG2);
    

    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
	scope.connectChannel(Iab.alpha, "Ialpha");
	scope.connectChannel(sine, "sine");
	scope.connectChannel(local_vgrid, "local_vgrid");
	scope.connectChannel(local_igrid, "local_igrid");
	scope.connectChannel(teaching_mode_scope, "teaching_mode");
    scope.connectChannel(Vdq.q, "Vq_in");
	scope.connectChannel(Vdq.d, "Vd_in");
    scope.connectChannel(Vdq_output.q, "Vq_out");
	scope.connectChannel(Vdq_output.d, "Vd_out");
	scope.connectChannel(Vab.alpha, "Valpha");
	scope.connectChannel(Vab.beta, "Vbeta");
	scope.connectChannel(Valpha_in_out, "Valpha(out-in)");
	scope.connectChannel(Vab_output.alpha, "ValphaOut");
	scope.connectChannel(Vab_output.beta, "VbetaOut");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();

    inverter.init(local_mode, Udc, Vgrid_amplitude_ref, w0, Ts);

    Idq_ref.d = 0.0;
    Idq_ref.q = 0.0;
    Vdq_ref.d = Vgrid_amplitude_ref;
    Vdq_ref.q = 0.0;

    Idq_ref_max.d = 8.0;
    Idq_ref_max.q = 1.0;
    Idq_ref_min.d = -0.1;
    Idq_ref_min.q = -0.1;

    Vdq_ref_max.d = 30.0;
    Vdq_ref_max.q = 30.0;
    Vdq_ref_min.d = -0.1;
    Vdq_ref_min.q = -0.1;



    Idq_ref_delta.d = 0.0;
    Idq_ref_delta.q = 0.0;



	is_net_synchronized = false;

    /* buck voltage mode */
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);
    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, control_task_period); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task


}

//--------------LOOP FUNCTIONS--------------------------------

void loop_communication_task()
{
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- grid forming ------        |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press 1 : open-loop sine PWM       |\n");
            printk("|     press 2 : forming, local sine      |\n");
            printk("|     press 3 : following, local PLL     |\n");
            printk("|     press 4 : following, measured PLL  |\n");
            printk("|     press d : vdref up by 5V           |\n");
            printk("|     press c : vdref down by 5V         |\n");
            printk("|     press u : vdref up by 1V           |\n");
            printk("|     press j : vdref down by 1V         |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode_asked = IDLEMODE;
            break;
        case 'p':
                if (!is_downloading){
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
                if(local_mode == FORMING){
                    if (Vdq_ref.d < Vdq_ref_max.d)
                    {
                        Vdq_ref.d += 1.0F;
                    }
                }else{
                    if (Idq_ref.d < Idq_ref_max.d)
                    {
                        Idq_ref.d += 0.1F;
                    }                    
                }
            break;
        case 'j':
                if(local_mode == FORMING){
                    if (Vdq_ref.d > Vdq_ref_min.d)
                    {
                        Vdq_ref.d -= 1.0F;
                    }
                }else{
                    if (Idq_ref.d > Idq_ref_min.d)
                    {
                        Idq_ref.d -= 0.1F;
                    }                    
                }

            break;
        case 'd':
                if(local_mode == FORMING){
                    if (Vdq_ref.d < Vdq_ref_max.d)
                    {
                        Vdq_ref.d += 5.0F;
                    }
                }else{
                    if (Idq_ref.d < Idq_ref_max.d)
                    {
                        Idq_ref.d -= 1.0F;
                    }                    
                }
            break;
        case 'c':
                if(local_mode == FORMING){
                    if (Vdq_ref.d > Vdq_ref_min.d)
                    {
                        Vdq_ref.d += 5.0F;
                    }
                }else{
                    if (Idq_ref.d > Idq_ref_min.d)
                    {
                        Idq_ref.d -= 1.0F;
                    }                    
                }
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
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
/* --- STATE MACHINE --------------------------------------------------------*/
// mode is the STATE variable
// in each state we compute the transitions
switch (mode) {
        case IDLEMODE:

            if (local_mode == FORMING){
                if (mode_asked == POWERMODE && V_high_filt >= UDC_STARTUP) {
                    mode = STARTUPMODE;
                } 
            }else{
                if (mode_asked == POWERMODE && Vgrid_meas >= 10 && V_high_filt >= UDC_STARTUP) {
                    mode = STARTUPMODE;
                }
            }
            spin.led.turnOn();
        break;
        case STARTUPMODE:
            if (local_mode == FORMING && delta_duty_cycle > 0.49F )
            {
                mode = POWERMODE;
            } 
            else if(local_mode == FOLLOWING && is_net_synchronized == true) 
            {
                mode = POWERMODE;
                if (is_net_synchronized) spin.led.toggle();
            }

        break;
        case POWERMODE:
            if (mode_asked == IDLEMODE) {
                mode = IDLEMODE;
            }
            if (is_net_synchronized) spin.led.toggle();
        break;
        case ERRORMODE:
        break;
    }
    if (mode_asked == IDLEMODE) mode = IDLEMODE; // global return to idle possible
/* --- END OF STATE MACHINE -------------------------------------------------*/

    if (mode == IDLEMODE)
    {
        if (!is_downloading) {
            printk("%d:", mode);
            printk("% 7.3f:", (double)Vgrid_amplitude_ref);
            printk("% 7.3f:", (double)I1_low_value);
            printk("% 7.3f:", (double)I2_low_value);
            printk("% 7.3f:", (double)V1_low_value);
            printk("% 7.3f:", (double)V_high);
            printk("%7.3f:", (double)power.d);
            printk("%7.3f:", (double)power.q);
			printk("%7.3f:", (double)Idq_ref.d);
			printk("%7.3f:", (double)VN_meas);
            printk("\n");
        } else {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else
    {
	    printk("Mode %d:", mode);
	    printk("W %.0f:", omega);
            printk("% 7.3f:", (double)Vgrid_amplitude_ref);
	    printk("V1 % 6.2f:", (double)V1_low_value);

		printk("Vd_ref %7.3f:", (double)Vdq_ref.d);
		printk("Vd_in %7.3f:", (double)Vdq.d);
		printk("Vd_out %7.3f|", (double)Vdq_output.d);

		printk("Vq_ref %7.3f:", (double)Vdq_ref.q);
		printk("Vq_in %7.3f:", (double)Vdq.q);
		printk("Vq_out %7.3f|", (double)Vdq_output.q);
        

		printk("Id_delta %7.3f:", (double)Idq_ref_delta.d);
		printk("Id_in  %7.3f:", (double)Idq.d);
		printk("Id_ref %7.3f|", (double)Idq_ref.d);
        printk("Vdc %7.2f:", (double)V_high_filt);
        printk("\n");
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 100 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
    critical_task_counter++;

    // RETRIEVE MEASUREMENTS
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

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);

    Vgrid_meas = V1_low_value-V2_low_value;
    VN_meas = (V1_low_value+V2_low_value)/2;
    Igrid_meas = I1_low_value;
    // Igrid_meas = I1_low_value;
    update_teaching_sine();

    // MANAGE OVERCURRENT
    if (I1_low_value > MAX_CURRENT
        || I1_low_value < -MAX_CURRENT
        || I2_low_value > MAX_CURRENT
        || I2_low_value < -MAX_CURRENT)
    {
        mode = ERRORMODE;
    }


    if (mode == IDLEMODE || mode == ERRORMODE)
    {
        // FIRST WE STOP THE PWM
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
            spin.led.turnOff();
            pwm_enable = false;
        }
        // duty_cycle = DUTY_MIN;
    }

    if (mode == STARTUPMODE) { // ramp up the common voltage to Udc/2

        if(local_mode == FORMING){

            delta_duty_cycle = rate_limiter(0.5F, delta_duty_cycle, 50.0F); // ramp of 50/s
            if (delta_duty_cycle > 0.5F) {
                delta_duty_cycle = 0.5F;
            }
            duty_cycle_1 = delta_duty_cycle;
            duty_cycle_2 = delta_duty_cycle;
            shield.power.setDutyCycle(LEG2, delta_duty_cycle);
            shield.power.setDutyCycle(LEG1, delta_duty_cycle);
            // WE START THE PWM
            if (!pwm_enable)
            {
                shield.power.start(ALL);
                pwm_enable = true;
            }

        } else {
            inverter.setVBus(control_bus_voltage());
            inverter.setPowerOn(false);
            delta_duty_cycle = inverter.calculateDuty(following_vgrid_input(), following_igrid_input());
            Vdq = inverter.getVdq();
            omega = inverter.getw();
            is_net_synchronized = inverter.getSync() && following_frequency_in_range();

        }
    }

    if (mode == POWERMODE)
    {
        // trigger = true;
        // theta = ot_modulo_2pi(theta + w0 * Ts);
        // sine = (1 + 0.1*Vdq_ref.d*ot_sin(theta))/2;

        inverter.setVBus(V_high_filt);

        if (local_mode == FORMING ){
            if (teaching_mode == OPEN_LOOP) {
                delta_duty_cycle = 0.5F + local_vgrid / (2.0F * control_bus_voltage());
                apply_complementary_duty(delta_duty_cycle);
            } else {
                inverter.setVBus(control_bus_voltage());
                inverter.setVdqRef(Vdq_ref);
                delta_duty_cycle = inverter.calculateDuty(local_vgrid, local_igrid);
                apply_complementary_duty(delta_duty_cycle);
            }
        }else{
            inverter.setVBus(control_bus_voltage());
            inverter.setIdqRef(Idq_ref);
            inverter.setPowerOn(true);
            delta_duty_cycle = inverter.calculateDuty(following_vgrid_input(), following_igrid_input());
            omega = inverter.getw();
            is_net_synchronized = inverter.getSync() && following_frequency_in_range();
            handle_following_desync();
            if (is_net_synchronized)
            {
                apply_complementary_duty(delta_duty_cycle);
                if (!pwm_enable)
                {
                    shield.power.start(ALL);
                    pwm_enable = true;
                }
            }
            else
            {
                inverter.setPowerOn(false);
            }
        }

    }

    /* Retrieve multiple data for debugging */
    theta = inverter.getTheta();
    Vdq = inverter.getVdq();
    Vdq_output = inverter.getVdqOut();
    Vab = inverter.getVab();
    Vab_output = inverter.getVabOutput();
    Iab = inverter.getIab();
    Idq = inverter.getIdq();
    Idq_ref_delta = inverter.getIdqRefDelta();
    omega = inverter.getw();
    Valpha_in_out = Vab_output.alpha - Vab.alpha; 


    if (critical_task_counter%decimation == 0) {
        spying_mode = (float32_t) mode;
        scope.acquire();
    }
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
