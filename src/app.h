#pragma once

#include "ScopeMimicry.h"
#include "ShieldAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "motor_control.h"
#include "trigo.h"

struct AppSetup {
	/* Setup-time configuration chosen before the control loop starts. */
	float32_t ac_current_limit = 4.0F;          /* AC current protection threshold. */
	float32_t dc_current_limit = 4.0F;          /* DC current protection threshold. */
	float32_t min_dc_voltage = 30.0F;           /* Minimum bus voltage accepted by control. */
	float32_t v_high_min = 5.0F;                /* Minimum filtered bus voltage to enter power mode. */
	float32_t Ts = 100.e-6F;                    /* Critical control-loop period in seconds. */
	uint32_t control_task_period = 100U;        /* Critical task period in microseconds. */
	uint32_t speed_loop_decimation = 10U;       /* Ratio between current and speed loop updates. */
	float32_t Ts_speed = 1.0e-3F;               /* Effective speed-loop period in seconds. */
	float32_t electrical_offset_step = PI / 180.0F; /* Offset tuning step in radians. */
	float32_t open_loop_speed_step = 1.0F;      /* Open-loop speed tuning step. */
	float32_t open_loop_vq_step = 0.1F;         /* Open-loop q-voltage tuning step. */
	float32_t speed_kp_step = 0.001F;           /* Speed-loop Kp tuning step. */
	float32_t speed_ki_step = 0.05F;            /* Speed-loop Ki tuning step. */
	float32_t nb_offset = 2000.0F;              /* Samples used for current-offset calibration. */
	float32_t speed_ref_step = 1.0F;            /* Speed reference increment from serial commands. */
	float32_t speed_ref_max = 300.0F;           /* Absolute speed reference limit. */
	float32_t iq_ref_max = 4.0F;                /* Maximum q-axis current reference. */
	uint32_t decimation = 20U;                  /* Scope/logging decimation factor. */
};

struct AppVariable {
	/* Values updated by setup helpers, task handlers, or the control loop. */
	float32_t angle_filtered = 0.0F;            /* Electrical angle used by the controller. */
	float32_t w_meas = 0.0F;                    /* Filtered electrical speed estimate. */

	float32_t meas_data = 0.0F;                 /* Scratch value for latest sensor readback. */
	float32_t I1_low_value = 0.0F;              /* Phase-current measurement on LEG1. */
	float32_t I2_low_value = 0.0F;              /* Phase-current measurement on LEG2. */
	float32_t I1_offset = 0.0F;                 /* Calibration offset applied to LEG1 current. */
	float32_t I2_offset = 0.0F;                 /* Calibration offset applied to LEG2 current. */
	float32_t tmpI1_offset = 0.0F;              /* Accumulator used while calibrating LEG1 current. */
	float32_t tmpI2_offset = 0.0F;              /* Accumulator used while calibrating LEG2 current. */
	float32_t V1_low_value = 0.0F;              /* Phase voltage sample on LEG1. */
	float32_t V2_low_value = 0.0F;              /* Phase voltage sample on LEG2. */
	float32_t V12_value = 0.0F;                 /* Differential phase voltage used for scope output. */

	float32_t I_high = 0.0F;                    /* DC bus current measurement. */
	float32_t V_high = 0.0F;                    /* Raw DC bus voltage measurement. */

	uint32_t encoder_count = 0U;                /* Latest incremental encoder count. */
	uint32_t encoder_count_prev = 0U;           /* Previous incremental encoder count. */
	int32_t encoder_delta_count = 0;            /* Wrapped encoder count delta per control step. */
	float32_t encoder_mech_angle = 0.0F;        /* Mechanical angle reported by the position API. */
	float32_t encoder_elec_angle = 0.0F;        /* Electrical angle reported by the position API. */
	float32_t encoder_mech_speed = 0.0F;        /* Mechanical speed reported by the position API. */
	float32_t encoder_elec_speed = 0.0F;        /* Electrical speed reported by the position API. */
	position_sensor_type_t active_position_type = POSITION_SENSOR_TYPE_UNDEFINED; /* Selected position sensor type. */
	bool position_sensor_initialized = false;   /* True once the selected position sensor is ready. */
	bool position_data_valid = false;           /* True when the last position update succeeded. */

	three_phase_t Vabc = {};                    /* Three-phase voltage command. */
	three_phase_t duty_abc = {};                /* Three-phase PWM duty command. */
	three_phase_t Iabc = {};                    /* Three-phase current estimate from control. */
	dqo_t Vdq = {};                             /* Voltage command in dq frame. */
	dqo_t Idq = {};                             /* Measured current in dq frame. */
	dqo_t Idq_ref = {};                         /* Current reference in dq frame. */
	MotorControlOutput motor_output = {};       /* Latest output from the motor-control library. */
	float32_t angle_4_control = 0.0F;           /* Electrical angle used for dq/abc transforms. */

	three_phase_t Iabc_ref = {};                /* Phase-current reference mirrored for logging. */
	float32_t duty_a = 0.0F;                    /* Logged duty cycle on phase A. */
	float32_t duty_b = 0.0F;                    /* Logged duty cycle on phase B. */
	float32_t Ia_ref = 0.0F;                    /* Logged phase-A current reference. */
	float32_t Ib_ref = 0.0F;                    /* Logged phase-B current reference. */
	float32_t Va = 0.0F;                        /* Logged phase-A voltage command. */
	float32_t Iq_meas = 0.0F;                   /* Logged measured q-axis current. */
	float32_t Iq_ref = 0.0F;                    /* Logged q-axis current reference. */
	float32_t Id_ref = 0.0F;                    /* Logged d-axis current reference. */
	float32_t Iq_max = 0.0F;                    /* Active q-axis current limit used for display. */
	float32_t Vd = 0.0F;                        /* Logged d-axis voltage command. */
	float32_t Vq = 0.0F;                        /* Logged q-axis voltage command. */
	float32_t speed_ref = 0.0F;                 /* Requested electrical speed reference. */
	float32_t speed_ref_print = 0.0F;           /* Mirrored speed reference for scope output. */
	float32_t speed_meas_print = 0.0F;          /* Mirrored measured speed for scope output. */
	float32_t electrical_offset_print = 0.0F;   /* Mirrored electrical offset for scope output. */
	float32_t open_loop_mode_print = 0.0F;      /* Mirrored open-loop flag for scope output. */
	float32_t theta_ol_print = 0.0F;            /* Mirrored open-loop angle for scope output. */
	float32_t omega_ol_print = 0.0F;            /* Mirrored open-loop speed for scope output. */
	float32_t vq_ol_print = 0.0F;               /* Mirrored open-loop q-voltage for scope output. */
	float32_t iq_ref_from_speed = 0.0F;         /* Q-current request generated by the speed loop. */
	float32_t encoder_count_f = 0.0F;           /* Floating-point mirror of encoder count. */
	float32_t encoder_delta_count_f = 0.0F;     /* Floating-point mirror of encoder delta count. */

	LowPassFirstOrderFilter vHigh_filter =
		controlLibFactory.lowpassfilter(100.e-6F, 5.0e-3F); /* Bus-voltage low-pass filter. */
	LowPassFirstOrderFilter w_mes_filter =
		controlLibFactory.lowpassfilter(100.e-6F, 5.0e-3F); /* Electrical-speed low-pass filter. */
	float32_t V_high_filtered = 0.0F;           /* Filtered DC bus voltage. */
	float32_t speed_Kp = 0.01F;                 /* Current speed-loop proportional gain. */
	float32_t speed_Ti = 0.1F;                  /* Current speed-loop integral time. */
	float32_t speed_Ki = 0.1F;                  /* Current speed-loop integral gain. */
	uint8_t speed_decimation = 10U;             /* Runtime ABZ speed decimation setting. */
	MotorControl motor_control;                 /* Motor-control library instance. */

	uint32_t counter_time = 0U;                 /* Control-loop tick counter. */
	float32_t counter_time_f = 0.0F;            /* Floating-point mirror of tick counter. */
	uint8_t received_serial_char = 0U;          /* Latest serial command received. */
	uint16_t error_counter = 0U;                /* Accumulated protection fault counter. */
	bool pwm_enable = false;                    /* True once PWM outputs are enabled. */
	bool open_loop_mode = false;                /* True when synthetic angle control is enabled. */
	float32_t theta_ol = 0.0F;                  /* Open-loop electrical angle. */
	float32_t omega_ol = 10.0F;                 /* Open-loop electrical speed command. */
	float32_t vq_ol = 0.5F;                     /* Open-loop q-axis voltage command. */
	uint16_t k_app_idx = 0U;                    /* Replay index inside the scope buffer. */
	bool is_downloading = false;                /* True when a scope dump was requested. */
	bool memory_print = false;                  /* True when replaying buffered scope data. */
};

struct AppContext {
	AppSetup setup;                             /* Setup-time constants and configuration. */
	AppVariable variable;                       /* Live variables updated during execution. */
};

enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

enum control_state_mode {
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

/**
 * @brief Reset variable-owned variables to their startup defaults.
 *
 * This function initializes the mutable part of the application context before
 * the control tasks start running.
 *
 * @param app Application context containing setup and variable data.
 */
void initialize_variable_defaults(AppContext &app);

/**
 * @brief Configure the motor-control library from setup and tuning values.
 *
 * This function builds the motor-control configuration from `AppSetup` and the
 * current variable tuning values, then pushes it into the control library.
 *
 * @param app Application context containing setup and variable data.
 */
void init_motor_control(AppContext &app);

/**
 * @brief Reset filters, controller state, and fault counters.
 *
 * This function is used when leaving active power control or when reinitializing
 * the controller state after a mode change.
 *
 * @param app Application context containing setup and variable data.
 */
void init_filters_and_regulators(AppContext &app);

/**
 * @brief Restart current-offset calibration from a clean stopped state.
 *
 * This helper resets the current-offset accumulators, returns the command mode
 * to idle, and places the state machine back into the offset-calibration state.
 *
 * @param app Application context containing setup and variable data.
 * @param asked_mode Reference to the current requested operating mode.
 * @param idle_mode Value representing the idle command mode.
 * @param control_state Reference to the current control state.
 */
void restart_offset_calibration(AppContext &app,
								uint8_t &asked_mode,
								uint8_t idle_mode,
								control_state_mode &control_state);

/**
 * @brief Read analog sensors and update filtered measurement values.
 *
 * This function refreshes the current, voltage, and bus measurements from the
 * shield sensors and updates the filtered DC-bus voltage.
 *
 * @param app Application context containing setup and variable data.
 * @param offset_calibration_active True when current-offset accumulation is active.
 */
void retrieve_analog_data(AppContext &app, bool offset_calibration_active);

/**
 * @brief Update rotor position, speed, and encoder-derived quantities.
 *
 * This function queries the active position sensor, validates the returned
 * values, and updates the angle and speed variables used by the controller.
 *
 * @param app Application context containing setup and variable data.
 */
void update_position_and_speed(AppContext &app);

/**
 * @brief Count overcurrent events and report whether the error threshold was reached.
 *
 * This function inspects the measured currents and increments the fault counter
 * when protection thresholds are exceeded.
 *
 * @param app Application context containing setup and variable data.
 * @return `true` when the accumulated overcurrent counter crossed the fault threshold.
 */
bool update_overcurrent_error(AppContext &app);

/**
 * @brief Stop PWM if active and clear non-persistent control state.
 *
 * This function is used when the state machine is outside active power mode and
 * the inverter must be returned to a safe stopped state.
 *
 * @param app Application context containing setup and variable data.
 */
void stop_pwm_and_reset_states_if_needed(AppContext &app);

/**
 * @brief Refresh the variables mirrored to the scope and debug outputs.
 *
 * This helper copies the latest control outputs, references, and state values
 * into the variables used by `ScopeMimicry` and serial logging.
 *
 * @param app Application context containing setup and variable data.
 * @param control_state Current control-state value to mirror for logging.
 * @param control_state_f Floating-point mirror of the control state.
 */
void update_scope_mirrors(AppContext &app,
						  control_state_mode control_state,
						  float32_t &control_state_f);

/**
 * @brief Run the closed-loop current and speed control path.
 *
 * This function feeds measured quantities into the motor-control library while
 * in normal closed-loop operation and stores the resulting commands.
 *
 * @param app Application context containing setup and variable data.
 */
void control_speed(AppContext &app);

/**
 * @brief Run the open-loop voltage generation path.
 *
 * This function drives the motor-control library in open-loop mode using a
 * synthetic electrical angle trajectory.
 *
 * @param app Application context containing setup and variable data.
 */
void control_open_loop(AppContext &app);

/**
 * @brief Keep a dedicated hook for duty computation staging.
 *
 * This helper preserves the separation between control-law evaluation and PWM
 * application even when duties are already produced by the motor-control library.
 *
 * @param app Application context containing setup and variable data.
 */
void compute_duties(AppContext &app);

/**
 * @brief Apply the current duty commands to the three power legs.
 *
 * This function forwards the duty-cycle commands stored in the application
 * context to the shield power API.
 *
 * @param app Application context containing setup and variable data.
 */
void apply_duties(const AppContext &app);

/**
 * @brief Start PWM outputs once when entering active power mode.
 *
 * This function ensures that PWM generation is started only once when the
 * state machine transitions into active inverter operation.
 *
 * @param app Application context containing setup and variable data.
 */
void start_pwms_if_needed(AppContext &app);

/**
 * @brief Adjust the stored electrical offset on the active position sensor.
 *
 * This helper shifts the electrical offset used by the position API and prints
 * the updated value to the serial console.
 *
 * @param app Application context containing setup and variable data.
 * @param delta Offset increment to apply, in radians.
 */
void adjust_electrical_offset(AppContext &app, float32_t delta);

/**
 * @brief Toggle between closed-loop and open-loop motor-control modes.
 *
 * This function switches the control library mode, synchronizes the open-loop
 * angle with the measured position, and resets the controller state.
 *
 * @param app Application context containing setup and variable data.
 */
void toggle_open_loop_mode(AppContext &app);

/**
 * @brief Update the speed-loop proportional gain and refresh derived tuning values.
 *
 * This helper adjusts the proportional gain of the outer speed loop and then
 * reads back the effective controller gains from the motor-control library.
 *
 * @param app Application context containing setup and variable data.
 * @param delta Gain increment to apply to the proportional term.
 */
void adjust_speed_loop_kp(AppContext &app, float32_t delta);

/**
 * @brief Update the speed-loop integral gain and refresh derived tuning values.
 *
 * This helper adjusts the integral gain of the outer speed loop and then reads
 * back the effective controller gains from the motor-control library.
 *
 * @param app Application context containing setup and variable data.
 * @param delta Gain increment to apply to the integral term.
 */
void adjust_speed_loop_ki(AppContext &app, float32_t delta);

/**
 * @brief Stream the full captured scope buffer over the serial console.
 *
 * This function dumps all stored `ScopeMimicry` samples in sequence, using the
 * background task delay to avoid saturating the serial path.
 *
 * @param scope Scope buffer to dump over the serial console.
 */
void dump_scope_datas(ScopeMimicry &scope);
