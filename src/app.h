#pragma once

#include "ScopeMimicry.h"
#include "ShieldAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "motor_control.h"
#include "trigo.h"

struct AppSetup {
	/* Setup-time configuration chosen before the control loop starts. */
	float32_t ac_current_limit = 4.0F;
	float32_t dc_current_limit = 4.0F;
	float32_t min_dc_voltage = 30.0F;
	float32_t v_high_min = 5.0F;
	float32_t Ts = 100.e-6F;
	uint32_t control_task_period = 100U;
	uint32_t speed_loop_decimation = 10U;
	float32_t Ts_speed = 1.0e-3F;
	float32_t electrical_offset_step = PI / 180.0F;
	float32_t open_loop_speed_step = 1.0F;
	float32_t open_loop_vq_step = 0.1F;
	float32_t speed_kp_step = 0.001F;
	float32_t speed_ki_step = 0.05F;
	float32_t nb_offset = 2000.0F;
	float32_t speed_ref_step = 1.0F;
	float32_t speed_ref_max = 300.0F;
	float32_t iq_ref_max = 4.0F;
	uint32_t decimation = 20U;
};

struct AppRuntime {
	/* Values updated by setup helpers, task handlers, or the control loop. */
	float32_t angle_filtered = 0.0F;
	float32_t w_meas = 0.0F;

	float32_t meas_data = 0.0F;
	float32_t I1_low_value = 0.0F;
	float32_t I2_low_value = 0.0F;
	float32_t I1_offset = 0.0F;
	float32_t I2_offset = 0.0F;
	float32_t tmpI1_offset = 0.0F;
	float32_t tmpI2_offset = 0.0F;
	float32_t V1_low_value = 0.0F;
	float32_t V2_low_value = 0.0F;
	float32_t V12_value = 0.0F;

	float32_t I_high = 0.0F;
	float32_t V_high = 0.0F;

	uint32_t encoder_count = 0U;
	uint32_t encoder_count_prev = 0U;
	int32_t encoder_delta_count = 0;
	float32_t encoder_mech_angle = 0.0F;
	float32_t encoder_elec_angle = 0.0F;
	float32_t encoder_mech_speed = 0.0F;
	float32_t encoder_elec_speed = 0.0F;
	position_sensor_type_t active_position_type = POSITION_SENSOR_TYPE_UNDEFINED;
	bool position_sensor_initialized = false;
	bool position_data_valid = false;

	three_phase_t Vabc = {};
	three_phase_t duty_abc = {};
	three_phase_t Iabc = {};
	dqo_t Vdq = {};
	dqo_t Idq = {};
	dqo_t Idq_ref = {};
	MotorControlOutput motor_output = {};
	float32_t angle_4_control = 0.0F;

	three_phase_t Iabc_ref = {};
	float32_t duty_a = 0.0F;
	float32_t duty_b = 0.0F;
	float32_t Ia_ref = 0.0F;
	float32_t Ib_ref = 0.0F;
	float32_t Va = 0.0F;
	float32_t Iq_meas = 0.0F;
	float32_t Iq_ref = 0.0F;
	float32_t Id_ref = 0.0F;
	float32_t Iq_max = 0.0F;
	float32_t Vd = 0.0F;
	float32_t Vq = 0.0F;
	float32_t speed_ref = 0.0F;
	float32_t speed_ref_print = 0.0F;
	float32_t speed_meas_print = 0.0F;
	float32_t electrical_offset_print = 0.0F;
	float32_t open_loop_mode_print = 0.0F;
	float32_t theta_ol_print = 0.0F;
	float32_t omega_ol_print = 0.0F;
	float32_t vq_ol_print = 0.0F;
	float32_t iq_ref_from_speed = 0.0F;
	float32_t encoder_count_f = 0.0F;
	float32_t encoder_delta_count_f = 0.0F;

	LowPassFirstOrderFilter vHigh_filter =
		controlLibFactory.lowpassfilter(100.e-6F, 5.0e-3F);
	LowPassFirstOrderFilter w_mes_filter =
		controlLibFactory.lowpassfilter(100.e-6F, 5.0e-3F);
	float32_t V_high_filtered = 0.0F;
	float32_t speed_Kp = 0.01F;
	float32_t speed_Ti = 0.1F;
	float32_t speed_Ki = 0.1F;
	uint8_t speed_decimation = 10U;
	MotorControl motor_control;

	uint32_t counter_time = 0U;
	float32_t counter_time_f = 0.0F;
	uint8_t received_serial_char = 0U;
	uint16_t error_counter = 0U;
	bool pwm_enable = false;
	bool open_loop_mode = false;
	float32_t theta_ol = 0.0F;
	float32_t omega_ol = 10.0F;
	float32_t vq_ol = 0.5F;
	uint16_t k_app_idx = 0U;
	bool is_downloading = false;
	bool memory_print = false;
};

struct AppContext {
	AppSetup setup;
	AppRuntime runtime;
};

void initialize_runtime_defaults(AppContext &app);
void init_motor_control(AppContext &app);
void init_filters_and_regulators(AppContext &app);
void retrieve_analog_data(AppContext &app, bool offset_calibration_active);
void update_position_and_speed(AppContext &app);
bool update_overcurrent_error(AppContext &app);
void stop_pwm_and_reset_states_if_needed(AppContext &app);
void control_speed(AppContext &app);
void control_open_loop(AppContext &app);
void compute_duties(AppContext &app);
void apply_duties(const AppContext &app);
void start_pwms_if_needed(AppContext &app);
void adjust_electrical_offset(AppContext &app, float32_t delta);
void toggle_open_loop_mode(AppContext &app);
void adjust_speed_loop_kp(AppContext &app, float32_t delta);
void adjust_speed_loop_ki(AppContext &app, float32_t delta);
void dump_scope_datas(ScopeMimicry &scope);
