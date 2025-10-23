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
// #include "power_ac1phase.h"
#include "ScopeMimicry.h"
#include "control_factory.h"
#include "zephyr/console/console.h"
#include "singlePhaseInverter.h"
#include "sogi.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F
#define UDC_STARTUP 0.0F
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine();           /* Setups the hardware and software of the system */

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

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

// static PowerAC1PhaseOutput pq_power;
// static PowerAC1PhaseParams ac_meas_config;
// static PowerAC1Phase inverter;

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




static float32_t Id_ref_delta = 0.0;
static float32_t Iq_ref_delta = 0.0;


static float32_t Vd_ref_max = 20.0;
static float32_t Vd_ref_min = 0.0;


static clarke_t Vab;
static clarke_t Vab_output;
static clarke_t Iab;

static float32_t Vond;
static float32_t R_load = 10;

static float32_t Ialpha, Ibeta;
static bool is_net_synchronized;
static float32_t omega;

static inverter_mode local_mode = FORMING;

/* duty_cycle*/
static float32_t delta_duty_cycle;// [No unit]
static float32_t duty_cycle_1;// [No unit]
static float32_t duty_cycle_2;// [No unit]
static float32_t duty_cycle_offset;// [No unit]

static float32_t Udc = 20.0F; // dc voltage supply assumed [V]
static const float f0 = 50.0F; // fundamental frequency [Hz]
static const float32_t w0 = 2.0F * PI * f0;   // pulsation [rad/s]
static const float32_t sync_power_tolerance = 0.01*w0;
/* Sinewave settings */
static float32_t Vgrid_ref; //[V]
static float32_t Vgrid_amplitude_ref = 20.0F; // [V]
static float32_t Vgrid_amplitude = 20.0F; // [V]
static float angle = 0.F; // [rad]
static float theta = 0.F; // [rad]

//------------- PR RESONANT -------------------------------------
static float32_t Ts = control_task_period * 1.0e-6F;

// static float32_t kp = 0.000215;
// static float32_t Ti = 0.2*7.5175e-5;
static float32_t kp = 0.001;      // kp is very small due to the fact that we are on a pure delay system (ref Viking)
static float32_t Ti = 0.001/3000; // Ti is Kp/Ki
float32_t Td = 0.0;
float32_t N = 1.0;
float32_t upper_bound = Udc;
float32_t lower_bound = -Udc;

static Pid pi_current_d = controlLibFactory.pid(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pi_current_q = controlLibFactory.pid(Ts, kp, Ti, Td, N, lower_bound, upper_bound);

static Pid pi_voltage_d = controlLibFactory.pid(Ts, 0.01, 0.003, Td, N, lower_bound, upper_bound);
static Pid pi_voltage_q = controlLibFactory.pid(Ts, 0.01, 0.003, Td, N, lower_bound, upper_bound);

Sogi sogi_i;
Sogi sogi_v;


// comes from "filters.h"
LowPassFirstOrderFilter vHighFilter(Ts, 0.1F);
LowPassFirstOrderFilter VqFilter(Ts, 1.0F);
static uint32_t critical_task_counter;
static uint32_t decimation = 1;
static uint32_t sync_counter=0;
static uint32_t desync_counter=0;
static uint32_t power_counter=0;
static float32_t desync_counter_scope;
static bool sync_start_flag = false;
static float32_t Vq_filtered;

// the scope help us to record datas during the critical task
// its a library which must be included in platformio.ini
static ScopeMimicry scope(1024, 19);
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
    shield.sensors.enableDefaultTwistSensors();

    // DISABLE DC LOW CAPACITORS
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    

    scope.connectChannel(I1_low_value, "I1_low_value");
    // scope.connectChannel(I_high, "I_High");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    // scope.connectChannel(V1_low_value, "V1_low_value");
    // scope.connectChannel(V2_low_value, "V2_low_value");
    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
    // scope.connectChannel(duty_cycle_offset, "duty_cycle_offset");
    // scope.connectChannel(power.d, "power_p");
    // scope.connectChannel(power.q, "power_q");
    // scope.connectChannel(Vdq_ref.d, "Vd_ref");
    // scope.connectChannel(theta, "theta");
	// scope.connectChannel(I2_low_value, "I2_low_value");
	scope.connectChannel(Idq.d, "Id");
	scope.connectChannel(Idq.q, "Iq");
	scope.connectChannel(Idq_ref.d, "Id_ref");
	// scope.connectChannel(Idq_ref_delta.q, "Idelta_q");
    // scope.connectChannel(Idq_ref_delta.d, "Idelta_d");
	scope.connectChannel(Iab.alpha, "Ialpha");
	scope.connectChannel(Iab.beta, "Ibeta");

    
	// scope.connectChannel(VN_meas, "VN_meas");
	// scope.connectChannel(desync_counter_scope, "desync_counter");
    scope.connectChannel(Vdq.q, "Vq_in");
	scope.connectChannel(Vdq.d, "Vd_in");
    scope.connectChannel(Vdq_output.q, "Vq_out");
	scope.connectChannel(Vdq_output.d, "Vd_out");
	// scope.connectChannel(Vq_filtered, "Vq_filtered");
	scope.connectChannel(Vab.alpha, "Valpha");
	scope.connectChannel(Vab.beta, "Vbeta");
	scope.connectChannel(Valpha_in_out, "Valpha(out-in)");
	scope.connectChannel(Vab_output.alpha, "ValphaOut");
	scope.connectChannel(Vab_output.beta, "VbetaOut");
	// scope.connectChannel(omega, "omega");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();

    // PR initialisation.

    // ac_meas_config.grid_voltage = 10.0;
    // ac_meas_config.w0 = w0;
    // ac_meas_config.Ts = Ts;

    inverter.init(local_mode, Vgrid_amplitude_ref, w0, Ts);

    sogi_v.init(500.0, Ts);
    sogi_i.init(500.0, Ts);

    Idq_ref.d = 0.0;
    Idq_ref.q = 0.0;
    Vdq_ref.d = 0.0;
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



    // power_ac1phase_init(&ac_meas_config, 10.0, 2.0*PI*50.0, Ts);
	pi_current_d.reset();
	pi_current_q.reset();
	pi_voltage_d.reset();
	pi_voltage_q.reset();
	is_net_synchronized = false;

    /* buck voltage mode */
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);
    // shield.power.setPhaseShift(LEG2,90);

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
            inverter.inputProcessing(Vgrid_meas,Igrid_meas);             
            Vdq = inverter.getVdqIn();

            if (omega < w0 + sync_power_tolerance &&
                omega > w0 -sync_power_tolerance)
            {
                sync_counter++;
                if(sync_counter>2000){
                    is_net_synchronized = true;
                    sync_counter = 0;                                    
                }
            } else {
                sync_counter=0;
                is_net_synchronized = false;                
            }

        }
    }

    if (mode == POWERMODE)
    {
        inverter.inputProcessing(Vgrid_meas,Igrid_meas);
        
        if(local_mode == FOLLOWING){
            is_net_synchronized = omega <= w0 + sync_power_tolerance && 
                                omega >= w0 -sync_power_tolerance; 

            if (is_net_synchronized == false)
            {
                desync_counter++;
                desync_counter_scope = (float32_t)desync_counter;
                if(desync_counter > 200){
                    desync_counter = 0;
                    sync_counter = 0;
                    mode_asked = IDLEMODE;
                    mode = IDLEMODE;
                    printk("System no longer synchronized \n");
                }                
            }
        }


        inverter.setVBus(V_high_filt);

        if (local_mode == FORMING ){
            inverter.setVdqRef(Vdq_ref);
        }else{
            inverter.setIdqRef(Idq_ref);
        }

        delta_duty_cycle = inverter.calculateDuty();


        if (local_mode == FOLLOWING ){
            if(pwm_enable = false)
            {        
                duty_cycle_offset = VN_meas/V_high_filt;        
            } 
            else
            {
                if (duty_cycle_offset < 0.5F) {
                    duty_cycle_offset = rate_limiter(0.5F, duty_cycle_offset, 1.0F); // ramp of 0.1 duty / 100 ms = 0.1/0.1 = 1e-1/1e-1 = 1
                    
                } else {
                    duty_cycle_offset = 0.5F;
                }

            }
        } else {
            duty_cycle_offset = 0.5F;
        }

        duty_cycle_1 = delta_duty_cycle + duty_cycle_offset;
        duty_cycle_2 = - delta_duty_cycle + duty_cycle_offset ;
        
        if (local_mode == FOLLOWING && !pwm_enable)
        {
            power_counter++;
            if(power_counter>2000){
                shield.power.start(ALL);
                pwm_enable = true;
            }
        }



        shield.power.setDutyCycle(LEG1, duty_cycle_1);
        shield.power.setDutyCycle(LEG2, duty_cycle_2);

    }

    /* Retrieve multiple data for debugging */
    theta = inverter.getTheta();
    Vdq = inverter.getVdqIn();
    Vq_filtered = VqFilter.calculateWithReturn(Vdq.q);
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
