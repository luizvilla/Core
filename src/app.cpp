#include "app.h"

#include "TaskAPI.h"
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

void initialize_runtime_defaults(AppContext &app)
{
	app.runtime.vHigh_filter =
		controlLibFactory.lowpassfilter(app.setup.Ts, 5.0e-3F);
	app.runtime.w_mes_filter =
		controlLibFactory.lowpassfilter(app.setup.Ts, 5.0e-3F);
	app.runtime.angle_filtered = 0.0F;
	app.runtime.w_meas = 0.0F;
	app.runtime.meas_data = 0.0F;
	app.runtime.I1_low_value = 0.0F;
	app.runtime.I2_low_value = 0.0F;
	app.runtime.I1_offset = 0.0F;
	app.runtime.I2_offset = 0.0F;
	app.runtime.tmpI1_offset = 0.0F;
	app.runtime.tmpI2_offset = 0.0F;
	app.runtime.V1_low_value = 0.0F;
	app.runtime.V2_low_value = 0.0F;
	app.runtime.V12_value = 0.0F;
	app.runtime.I_high = 0.0F;
	app.runtime.V_high = 0.0F;
	app.runtime.encoder_count = 0U;
	app.runtime.encoder_count_prev = 0U;
	app.runtime.encoder_delta_count = 0;
	app.runtime.encoder_mech_angle = 0.0F;
	app.runtime.encoder_elec_angle = 0.0F;
	app.runtime.encoder_mech_speed = 0.0F;
	app.runtime.encoder_elec_speed = 0.0F;
	app.runtime.active_position_type = POSITION_SENSOR_TYPE_UNDEFINED;
	app.runtime.position_sensor_initialized = false;
	app.runtime.position_data_valid = false;
	app.runtime.Vabc = {};
	app.runtime.duty_abc = {};
	app.runtime.Iabc = {};
	app.runtime.Vdq = {};
	app.runtime.Idq = {};
	app.runtime.Iq_max = app.setup.iq_ref_max;
	app.runtime.Vd = 0.0F;
	app.runtime.Vq = 0.0F;
	app.runtime.speed_ref = 0.0F;
	app.runtime.speed_ref_print = 0.0F;
	app.runtime.speed_meas_print = 0.0F;
	app.runtime.electrical_offset_print = 0.0F;
	app.runtime.open_loop_mode_print = 0.0F;
	app.runtime.theta_ol_print = 0.0F;
	app.runtime.omega_ol_print = 10.0F;
	app.runtime.vq_ol_print = 0.5F;
	app.runtime.iq_ref_from_speed = 0.0F;
	app.runtime.encoder_count_f = 0.0F;
	app.runtime.encoder_delta_count_f = 0.0F;
	app.runtime.V_high_filtered = 0.0F;
	app.runtime.speed_Kp = 0.01F;
	app.runtime.speed_Ti = 0.1F;
	app.runtime.speed_Ki = 0.1F;
	app.runtime.speed_decimation = (uint8_t)app.setup.speed_loop_decimation;
	app.runtime.counter_time = 0U;
	app.runtime.counter_time_f = 0.0F;
	app.runtime.received_serial_char = 0U;
	app.runtime.error_counter = 0U;
	app.runtime.pwm_enable = false;
	app.runtime.open_loop_mode = false;
	app.runtime.theta_ol = 0.0F;
	app.runtime.omega_ol = 10.0F;
	app.runtime.vq_ol = 0.5F;
	app.runtime.k_app_idx = 0U;
	app.runtime.is_downloading = false;
	app.runtime.memory_print = false;
	app.runtime.Idq_ref.d = 0.0F;
	app.runtime.Idq_ref.q = 0.0F;
	app.runtime.Idq_ref.o = 0.0F;
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
	config.speed_pi_kp = app.runtime.speed_Kp;
	config.speed_pi_ti = app.runtime.speed_Ti;
	config.speed_loop_decimation = app.setup.speed_loop_decimation;

	(void)app.runtime.motor_control.init(config);
	app.runtime.motor_control.setCurrentReference(app.runtime.Idq_ref);
	app.runtime.motor_control.setSpeedReference(app.runtime.speed_ref);
	app.runtime.motor_control.setOpenLoopSpeed(app.runtime.omega_ol);
	app.runtime.motor_control.setOpenLoopVoltageQ(app.runtime.vq_ol);
}

void init_filters_and_regulators(AppContext &app)
{
	app.runtime.vHigh_filter.reset(app.setup.v_high_min);
	app.runtime.motor_control.reset();
	app.runtime.error_counter = 0U;
}

void retrieve_analog_data(AppContext &app, bool offset_calibration_active)
{
	app.runtime.meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.I1_low_value =
			app.runtime.meas_data + app.runtime.I1_offset;
	}

	app.runtime.meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.I2_low_value =
			app.runtime.meas_data + app.runtime.I2_offset;
	}

	if (offset_calibration_active &&
		app.runtime.counter_time < (uint32_t)app.setup.nb_offset) {
		app.runtime.tmpI1_offset += app.runtime.I1_low_value;
		app.runtime.tmpI2_offset += app.runtime.I2_low_value;
	}

	app.runtime.meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.V_high = app.runtime.meas_data;
	}

	app.runtime.meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.I_high = -app.runtime.meas_data;
	}

	app.runtime.meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.V1_low_value = app.runtime.meas_data;
	}

	app.runtime.meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (app.runtime.meas_data != NO_VALUE) {
		app.runtime.V2_low_value = app.runtime.meas_data;
	}

	app.runtime.V_high_filtered =
		app.runtime.vHigh_filter.calculateWithReturn(app.runtime.V_high);
	app.runtime.V12_value =
		app.runtime.V1_low_value - app.runtime.V2_low_value;
}

void update_position_and_speed(AppContext &app)
{
	app.runtime.position_data_valid = false;
	if (!app.runtime.position_sensor_initialized) {
		return;
	}

	if (!shield.position.update(app.setup.Ts)) {
		return;
	}

	if (app.runtime.active_position_type == ABZ_TYPE) {
		uint32_t counts_per_revolution =
			shield.position.getCountsPerRevolution();
		app.runtime.encoder_count =
			shield.position.getIncrementalEncoderValue();
		app.runtime.encoder_delta_count = normalize_encoder_delta(
			app.runtime.encoder_count,
			app.runtime.encoder_count_prev,
			counts_per_revolution);
		app.runtime.encoder_count_prev = app.runtime.encoder_count;
	} else {
		app.runtime.encoder_count = 0U;
		app.runtime.encoder_delta_count = 0;
	}

	app.runtime.encoder_mech_angle = shield.position.getMechanicalAngle();
	app.runtime.encoder_elec_angle = shield.position.getElectricalAngle();
	app.runtime.encoder_mech_speed = shield.position.getMechanicalSpeed();
	app.runtime.encoder_elec_speed = shield.position.getElectricalSpeed();
	app.runtime.angle_filtered = app.runtime.encoder_elec_angle;
	app.runtime.w_meas = app.runtime.w_mes_filter.calculateWithReturn(
		app.runtime.encoder_elec_speed);
	app.runtime.position_data_valid = true;
}

bool update_overcurrent_error(AppContext &app)
{
	if (app.runtime.I1_low_value > app.setup.ac_current_limit ||
		app.runtime.I1_low_value < -app.setup.ac_current_limit ||
		app.runtime.I2_low_value > app.setup.ac_current_limit ||
		app.runtime.I2_low_value < -app.setup.ac_current_limit ||
		app.runtime.I_high > app.setup.dc_current_limit) {
		app.runtime.error_counter++;
	}

	return app.runtime.error_counter > 1000U;
}

void stop_pwm_and_reset_states_if_needed(AppContext &app)
{
	if (app.runtime.pwm_enable) {
		shield.power.stop(ALL);
		init_filters_and_regulators(app);
		app.runtime.pwm_enable = false;
	}

	app.runtime.theta_ol = 0.0F;
}

void control_speed(AppContext &app)
{
	app.runtime.Idq_ref.d = 0.0F;
	app.runtime.motor_control.setMode(MotorControlMode::Current);
	app.runtime.motor_control.setCurrentReference(app.runtime.Idq_ref);
	app.runtime.motor_control.setSpeedReference(app.runtime.speed_ref);

	MotorControlInput input;
	input.ia = app.runtime.I1_low_value;
	input.ib = app.runtime.I2_low_value;
	input.vbus = app.runtime.V_high_filtered;
	input.theta_elec = app.runtime.angle_filtered;
	input.omega_elec = app.runtime.w_meas;
	input.position_valid = app.runtime.position_data_valid;

	app.runtime.motor_output = app.runtime.motor_control.step(input);
	app.runtime.angle_4_control = app.runtime.motor_output.theta_control;
	app.runtime.Iabc = app.runtime.motor_output.iabc;
	app.runtime.Idq = app.runtime.motor_output.idq;
	app.runtime.Idq_ref = app.runtime.motor_output.idq_ref;
	app.runtime.Vdq = app.runtime.motor_output.vdq;
	app.runtime.Vabc = app.runtime.motor_output.vabc;
	app.runtime.duty_abc = app.runtime.motor_output.duty_abc;
	app.runtime.iq_ref_from_speed = app.runtime.motor_output.idq_ref.q;
}

void control_open_loop(AppContext &app)
{
	app.runtime.motor_control.setMode(MotorControlMode::OpenLoop);
	app.runtime.motor_control.setOpenLoopSpeed(app.runtime.omega_ol);
	app.runtime.motor_control.setOpenLoopVoltageQ(app.runtime.vq_ol);

	MotorControlInput input;
	input.ia = app.runtime.I1_low_value;
	input.ib = app.runtime.I2_low_value;
	input.vbus = app.runtime.V_high_filtered;
	input.theta_elec = app.runtime.angle_filtered;
	input.omega_elec = app.runtime.w_meas;
	input.position_valid = app.runtime.position_data_valid;

	app.runtime.motor_output = app.runtime.motor_control.step(input);
	app.runtime.theta_ol = app.runtime.motor_output.theta_control;
	app.runtime.angle_4_control = app.runtime.motor_output.theta_control;
	app.runtime.Iabc = app.runtime.motor_output.iabc;
	app.runtime.Idq = app.runtime.motor_output.idq;
	app.runtime.Idq_ref = app.runtime.motor_output.idq_ref;
	app.runtime.Vdq = app.runtime.motor_output.vdq;
	app.runtime.Vabc = app.runtime.motor_output.vabc;
	app.runtime.duty_abc = app.runtime.motor_output.duty_abc;
	app.runtime.iq_ref_from_speed = 0.0F;
}

void compute_duties(AppContext &app)
{
	(void)app;
}

void apply_duties(const AppContext &app)
{
	shield.power.setDutyCycle(LEG1, app.runtime.duty_abc.a);
	shield.power.setDutyCycle(LEG2, app.runtime.duty_abc.b);
	shield.power.setDutyCycle(LEG3, app.runtime.duty_abc.c);
}

void start_pwms_if_needed(AppContext &app)
{
	if (!app.runtime.pwm_enable) {
		app.runtime.pwm_enable = true;
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
	app.runtime.open_loop_mode = !app.runtime.open_loop_mode;
	app.runtime.theta_ol = app.runtime.encoder_elec_angle;
	app.runtime.motor_control.setOpenLoopAngle(app.runtime.theta_ol);
	app.runtime.motor_control.setMode(app.runtime.open_loop_mode
										 ? MotorControlMode::OpenLoop
										 : MotorControlMode::Current);
	init_filters_and_regulators(app);

	printk("open-loop mode %s, theta_ol = %.4f rad, omega_ol = %.2f rad/s, vq_ol = %.2f V\n",
		   app.runtime.open_loop_mode ? "enabled" : "disabled",
		   (double)app.runtime.theta_ol,
		   (double)app.runtime.omega_ol,
		   (double)app.runtime.vq_ol);
}

void adjust_speed_loop_kp(AppContext &app, float32_t delta)
{
	app.runtime.speed_Kp += delta;
	if (app.runtime.speed_Kp < 1.0e-6F) {
		app.runtime.speed_Kp = 1.0e-6F;
	}
	app.runtime.motor_control.setSpeedLoopKp(app.runtime.speed_Kp);
	app.runtime.motor_control.setSpeedLoopKi(app.runtime.speed_Ki);
	app.runtime.motor_control.reset();
	app.runtime.speed_Kp = app.runtime.motor_control.getSpeedLoopKp();
	app.runtime.speed_Ki = app.runtime.motor_control.getSpeedLoopKi();
	app.runtime.speed_Ti = app.runtime.motor_control.getSpeedLoopTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)app.runtime.speed_Kp,
		   (double)app.runtime.speed_Ki,
		   (double)app.runtime.speed_Ti);
}

void adjust_speed_loop_ki(AppContext &app, float32_t delta)
{
	app.runtime.speed_Ki += delta;
	if (app.runtime.speed_Ki < 1.0e-6F) {
		app.runtime.speed_Ki = 1.0e-6F;
	}
	app.runtime.motor_control.setSpeedLoopKi(app.runtime.speed_Ki);
	app.runtime.motor_control.reset();
	app.runtime.speed_Kp = app.runtime.motor_control.getSpeedLoopKp();
	app.runtime.speed_Ki = app.runtime.motor_control.getSpeedLoopKi();
	app.runtime.speed_Ti = app.runtime.motor_control.getSpeedLoopTi();
	printk("speed-loop Kp = %.4f, Ki = %.4f, Ti = %.4f\n",
		   (double)app.runtime.speed_Kp,
		   (double)app.runtime.speed_Ki,
		   (double)app.runtime.speed_Ti);
}
