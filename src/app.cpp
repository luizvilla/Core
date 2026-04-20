#include "app.h"

#include "TaskAPI.h"
#include "transform.h"
#include "trigo.h"

static int32_t normalize_encoder_delta(uint32_t current_count,
									   uint32_t previous_count,
									   uint32_t counts_per_revolution)
{
	int32_t delta = (int32_t)current_count - (int32_t)previous_count;

	if (counts_per_revolution == 0U) {
		return delta;
	}

	if (delta > ((int32_t)counts_per_revolution / 2)) {
		delta -= (int32_t)counts_per_revolution;
	} else if (delta < -((int32_t)counts_per_revolution / 2)) {
		delta += (int32_t)counts_per_revolution;
	}

	return delta;
}

void dump_scope_datas(ScopeMimicry &scope)
{
	printk("begin record\n");
	scope.reset_dump();
	while (scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

void initialize_variable_defaults(AppContext &app)
{
	app.variable.vHigh_filter =
		controlLibFactory.lowpassfilter(app.setup.Ts, 5.0e-3F);
	app.variable.w_mes_filter =
		controlLibFactory.lowpassfilter(app.setup.Ts, 5.0e-3F);
	app.variable.angle_filtered = 0.0F;
	app.variable.w_meas = 0.0F;
	app.variable.meas_data = 0.0F;
	app.variable.I1_low_value = 0.0F;
	app.variable.I2_low_value = 0.0F;
	app.variable.I1_offset = 0.0F;
	app.variable.I2_offset = 0.0F;
	app.variable.tmpI1_offset = 0.0F;
	app.variable.tmpI2_offset = 0.0F;
	app.variable.V1_low_value = 0.0F;
	app.variable.V2_low_value = 0.0F;
	app.variable.V12_value = 0.0F;
	app.variable.I_high = 0.0F;
	app.variable.V_high = 0.0F;
	app.variable.encoder_count = 0U;
	app.variable.encoder_count_prev = 0U;
	app.variable.encoder_delta_count = 0;
	app.variable.encoder_mech_angle = 0.0F;
	app.variable.encoder_elec_angle = 0.0F;
	app.variable.encoder_mech_speed = 0.0F;
	app.variable.encoder_elec_speed = 0.0F;
	app.variable.active_position_type = POSITION_SENSOR_TYPE_UNDEFINED;
	app.variable.position_sensor_initialized = false;
	app.variable.position_data_valid = false;
	app.variable.Vabc = {};
	app.variable.duty_abc = {};
	app.variable.Iabc = {};
	app.variable.Vdq = {};
	app.variable.Idq = {};
	app.variable.Iq_max = app.setup.iq_ref_max;
	app.variable.Vd = 0.0F;
	app.variable.Vq = 0.0F;
	app.variable.speed_ref = 0.0F;
	app.variable.speed_ref_print = 0.0F;
	app.variable.speed_meas_print = 0.0F;
	app.variable.electrical_offset_print = 0.0F;
	app.variable.open_loop_mode_print = 0.0F;
	app.variable.theta_ol_print = 0.0F;
	app.variable.omega_ol_print = 10.0F;
	app.variable.vq_ol_print = 0.5F;
	app.variable.iq_ref_from_speed = 0.0F;
	app.variable.encoder_count_f = 0.0F;
	app.variable.encoder_delta_count_f = 0.0F;
	app.variable.V_high_filtered = 0.0F;
	app.variable.speed_Kp = 0.01F;
	app.variable.speed_Ti = 0.1F;
	app.variable.speed_Ki = 0.1F;
	app.variable.speed_decimation = (uint8_t)app.setup.speed_loop_decimation;
	app.variable.counter_time = 0U;
	app.variable.counter_time_f = 0.0F;
	app.variable.received_serial_char = 0U;
	app.variable.error_counter = 0U;
	app.variable.pwm_enable = false;
	app.variable.open_loop_mode = false;
	app.variable.theta_ol = 0.0F;
	app.variable.omega_ol = 10.0F;
	app.variable.vq_ol = 0.5F;
	app.variable.k_app_idx = 0U;
	app.variable.is_downloading = false;
	app.variable.memory_print = false;
	app.variable.Idq_ref.d = 0.0F;
	app.variable.Idq_ref.q = 0.0F;
	app.variable.Idq_ref.o = 0.0F;
}

void init_motor_control(AppContext &app)
{
	MotorControlConfig config;
	config.Ts = app.setup.Ts;
	config.Ts_speed = app.setup.Ts_speed;
	config.min_bus_voltage = app.setup.min_dc_voltage;
	config.current_limit_q = app.setup.iq_ref_max;
	config.current_pi_kp = 30.0F * 0.035F;
	config.current_pi_ti = 0.002029F;
	config.speed_pi_kp = app.variable.speed_Kp;
	config.speed_pi_ti = app.variable.speed_Ti;
	config.speed_loop_decimation = app.setup.speed_loop_decimation;

	(void)app.variable.motor_control.init(config);
	app.variable.motor_control.setCurrentReference(app.variable.Idq_ref);
	app.variable.motor_control.setSpeedReference(app.variable.speed_ref);
	app.variable.motor_control.setOpenLoopSpeed(app.variable.omega_ol);
	app.variable.motor_control.setOpenLoopVoltageQ(app.variable.vq_ol);
}

void init_filters_and_regulators(AppContext &app)
{
	app.variable.vHigh_filter.reset(app.setup.v_high_min);
	app.variable.motor_control.reset();
	app.variable.error_counter = 0U;
}

void restart_offset_calibration(AppContext &app,
								uint8_t &asked_mode,
								uint8_t idle_mode,
								control_state_mode &control_state)
{
	stop_pwm_and_reset_states_if_needed(app);
	app.variable.counter_time = 0U;
	app.variable.encoder_count_prev = 0U;
	app.variable.I1_offset = 0.0F;
	app.variable.I2_offset = 0.0F;
	app.variable.tmpI1_offset = 0.0F;
	app.variable.tmpI2_offset = 0.0F;
	asked_mode = idle_mode;
	control_state = OFFSET_ST;
}

void retrieve_analog_data(AppContext &app, bool offset_calibration_active)
{
	app.variable.meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.I1_low_value =
			app.variable.meas_data + app.variable.I1_offset;
	}

	app.variable.meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.I2_low_value =
			app.variable.meas_data + app.variable.I2_offset;
	}

	if (offset_calibration_active &&
		app.variable.counter_time < (uint32_t)app.setup.nb_offset) {
		app.variable.tmpI1_offset += app.variable.I1_low_value;
		app.variable.tmpI2_offset += app.variable.I2_low_value;
	}

	app.variable.meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.V_high = app.variable.meas_data;
	}

	app.variable.meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.I_high = -app.variable.meas_data;
	}

	app.variable.meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.V1_low_value = app.variable.meas_data;
	}

	app.variable.meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (app.variable.meas_data != NO_VALUE) {
		app.variable.V2_low_value = app.variable.meas_data;
	}

	app.variable.V_high_filtered =
		app.variable.vHigh_filter.calculateWithReturn(app.variable.V_high);
	app.variable.V12_value =
		app.variable.V1_low_value - app.variable.V2_low_value;
}

void update_position_and_speed(AppContext &app)
{
	app.variable.position_data_valid = false;
	if (!app.variable.position_sensor_initialized) {
		return;
	}

	if (!shield.position.update(app.setup.Ts)) {
		return;
	}

	if (app.variable.active_position_type == ABZ_TYPE) {
		uint32_t counts_per_revolution =
			shield.position.getCountsPerRevolution();
		app.variable.encoder_count =
			shield.position.getIncrementalEncoderValue();
		app.variable.encoder_delta_count = normalize_encoder_delta(
			app.variable.encoder_count,
			app.variable.encoder_count_prev,
			counts_per_revolution);
		app.variable.encoder_count_prev = app.variable.encoder_count;
	} else {
		app.variable.encoder_count = 0U;
		app.variable.encoder_delta_count = 0;
	}

	app.variable.encoder_mech_angle = shield.position.getMechanicalAngle();
	app.variable.encoder_elec_angle = shield.position.getElectricalAngle();
	app.variable.encoder_mech_speed = shield.position.getMechanicalSpeed();
	app.variable.encoder_elec_speed = shield.position.getElectricalSpeed();
	app.variable.angle_filtered = app.variable.encoder_elec_angle;
	app.variable.w_meas = app.variable.w_mes_filter.calculateWithReturn(
		app.variable.encoder_elec_speed);
	app.variable.position_data_valid = true;
}

bool update_overcurrent_error(AppContext &app)
{
	if (app.variable.I1_low_value > app.setup.ac_current_limit ||
		app.variable.I1_low_value < -app.setup.ac_current_limit ||
		app.variable.I2_low_value > app.setup.ac_current_limit ||
		app.variable.I2_low_value < -app.setup.ac_current_limit ||
		app.variable.I_high > app.setup.dc_current_limit) {
		app.variable.error_counter++;
	}

	return app.variable.error_counter > 1000U;
}

void stop_pwm_and_reset_states_if_needed(AppContext &app)
{
	if (app.variable.pwm_enable) {
		shield.power.stop(ALL);
		init_filters_and_regulators(app);
		app.variable.pwm_enable = false;
	}

	app.variable.theta_ol = 0.0F;
}

void update_scope_mirrors(AppContext &app,
						  control_state_mode control_state,
						  float32_t &control_state_f)
{
	app.variable.encoder_count_f = (float32_t)app.variable.encoder_count;
	app.variable.encoder_delta_count_f = (float32_t)app.variable.encoder_delta_count;
	app.variable.Va = app.variable.Vabc.a;
	app.variable.duty_a = app.variable.duty_abc.a;
	app.variable.duty_b = app.variable.duty_abc.b;
	app.variable.Iq_ref = app.variable.Idq_ref.q;
	app.variable.Id_ref = app.variable.Idq_ref.d;
	app.variable.Iq_meas = app.variable.Idq.q;
	app.variable.Vd = app.variable.Vdq.d;
	app.variable.Vq = app.variable.Vdq.q;
	app.variable.speed_ref_print = app.variable.speed_ref;
	app.variable.speed_meas_print = app.variable.w_meas;
	app.variable.electrical_offset_print = shield.position.getElectricalOffset();
	app.variable.open_loop_mode_print = app.variable.open_loop_mode ? 1.0F : 0.0F;
	app.variable.theta_ol_print = app.variable.theta_ol;
	app.variable.omega_ol_print = app.variable.omega_ol;
	app.variable.vq_ol_print = app.variable.vq_ol;
	app.variable.Iabc_ref =
		Transform::to_threephase(app.variable.Idq_ref, app.variable.angle_4_control);
	app.variable.Ia_ref = app.variable.Iabc_ref.a;
	app.variable.Ib_ref = app.variable.Iabc_ref.b;
	app.variable.counter_time_f = (float32_t)app.variable.counter_time;
	control_state_f = (float32_t)control_state;
}

void control_speed(AppContext &app)
{
	app.variable.Idq_ref.d = 0.0F;
	app.variable.motor_control.setMode(MotorControlMode::Current);
	app.variable.motor_control.setCurrentReference(app.variable.Idq_ref);
	app.variable.motor_control.setSpeedReference(app.variable.speed_ref);

	MotorControlInput input;
	input.ia = app.variable.I1_low_value;
	input.ib = app.variable.I2_low_value;
	input.vbus = app.variable.V_high_filtered;
	input.theta_elec = app.variable.angle_filtered;
	input.omega_elec = app.variable.w_meas;
	input.position_valid = app.variable.position_data_valid;

	app.variable.motor_output = app.variable.motor_control.step(input);
	app.variable.angle_4_control = app.variable.motor_output.theta_control;
	app.variable.Iabc = app.variable.motor_output.iabc;
	app.variable.Idq = app.variable.motor_output.idq;
	app.variable.Idq_ref = app.variable.motor_output.idq_ref;
	app.variable.Vdq = app.variable.motor_output.vdq;
	app.variable.Vabc = app.variable.motor_output.vabc;
	app.variable.duty_abc = app.variable.motor_output.duty_abc;
	app.variable.iq_ref_from_speed = app.variable.motor_output.idq_ref.q;
}

void control_open_loop(AppContext &app)
{
	app.variable.motor_control.setMode(MotorControlMode::OpenLoop);
	app.variable.motor_control.setOpenLoopSpeed(app.variable.omega_ol);
	app.variable.motor_control.setOpenLoopVoltageQ(app.variable.vq_ol);

	MotorControlInput input;
	input.ia = app.variable.I1_low_value;
	input.ib = app.variable.I2_low_value;
	input.vbus = app.variable.V_high_filtered;
	input.theta_elec = app.variable.angle_filtered;
	input.omega_elec = app.variable.w_meas;
	input.position_valid = app.variable.position_data_valid;

	app.variable.motor_output = app.variable.motor_control.step(input);
	app.variable.theta_ol = app.variable.motor_output.theta_control;
	app.variable.angle_4_control = app.variable.motor_output.theta_control;
	app.variable.Iabc = app.variable.motor_output.iabc;
	app.variable.Idq = app.variable.motor_output.idq;
	app.variable.Idq_ref = app.variable.motor_output.idq_ref;
	app.variable.Vdq = app.variable.motor_output.vdq;
	app.variable.Vabc = app.variable.motor_output.vabc;
	app.variable.duty_abc = app.variable.motor_output.duty_abc;
	app.variable.iq_ref_from_speed = 0.0F;
}

void compute_duties(AppContext &app)
{
	(void)app;
}

void apply_duties(const AppContext &app)
{
	shield.power.setDutyCycle(LEG1, app.variable.duty_abc.a);
	shield.power.setDutyCycle(LEG2, app.variable.duty_abc.b);
	shield.power.setDutyCycle(LEG3, app.variable.duty_abc.c);
}

void start_pwms_if_needed(AppContext &app)
{
	if (!app.variable.pwm_enable) {
		app.variable.pwm_enable = true;
		shield.power.start(ALL);
	}
}

void adjust_electrical_offset(AppContext &app, float32_t delta)
{
	(void)app;
	float32_t current_offset = shield.position.getElectricalOffset();
	float32_t new_offset = ot_modulo_2pi(current_offset + delta);
	shield.position.setElectricalOffset(new_offset);

	printk("electrical offset = %.4f rad (%.1f deg)\n",
		   (double)new_offset,
		   (double)(new_offset * 180.0F / PI));
}

void toggle_open_loop_mode(AppContext &app)
{
	app.variable.open_loop_mode = !app.variable.open_loop_mode;
	app.variable.theta_ol = app.variable.encoder_elec_angle;
	app.variable.motor_control.setOpenLoopAngle(app.variable.theta_ol);
	app.variable.motor_control.setMode(app.variable.open_loop_mode
										 ? MotorControlMode::OpenLoop
										 : MotorControlMode::Current);
	init_filters_and_regulators(app);

	printk("open-loop mode %s, theta_ol = %.4f rad, omega_ol = %.2f rad/s, vq_ol = %.2f V\n",
		   app.variable.open_loop_mode ? "enabled" : "disabled",
		   (double)app.variable.theta_ol,
		   (double)app.variable.omega_ol,
		   (double)app.variable.vq_ol);
}

void adjust_speed_loop_kp(AppContext &app, float32_t delta)
{
	app.variable.speed_Kp += delta;
	if (app.variable.speed_Kp < 1.0e-6F) {
		app.variable.speed_Kp = 1.0e-6F;
	}
	app.variable.motor_control.setSpeedLoopKp(app.variable.speed_Kp);
	app.variable.motor_control.setSpeedLoopKi(app.variable.speed_Ki);
	app.variable.motor_control.reset();
	app.variable.speed_Kp = app.variable.motor_control.getSpeedLoopKp();
	app.variable.speed_Ki = app.variable.motor_control.getSpeedLoopKi();
	app.variable.speed_Ti = app.variable.motor_control.getSpeedLoopTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)app.variable.speed_Kp,
		   (double)app.variable.speed_Ki,
		   (double)app.variable.speed_Ti);
}

void adjust_speed_loop_ki(AppContext &app, float32_t delta)
{
	app.variable.speed_Ki += delta;
	if (app.variable.speed_Ki < 1.0e-6F) {
		app.variable.speed_Ki = 1.0e-6F;
	}
	app.variable.motor_control.setSpeedLoopKi(app.variable.speed_Ki);
	app.variable.motor_control.reset();
	app.variable.speed_Kp = app.variable.motor_control.getSpeedLoopKp();
	app.variable.speed_Ki = app.variable.motor_control.getSpeedLoopKi();
	app.variable.speed_Ti = app.variable.motor_control.getSpeedLoopTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)app.variable.speed_Kp,
		   (double)app.variable.speed_Ki,
		   (double)app.variable.speed_Ti);
}
