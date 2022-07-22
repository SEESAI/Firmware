/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "DragEstimator.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>

int DragEstimator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int DragEstimator::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int DragEstimator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("DragEstimator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1300,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DragEstimator *DragEstimator::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	DragEstimator *instance = new DragEstimator(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

DragEstimator::DragEstimator(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void DragEstimator::run()
{
	// Set default hover thrust estimate to 0.5
	_hover_thrust_estimate.hover_thrust = 0.5;

	parameters_update(true);

	while (!should_exit()) {
		px4_usleep(1000); // 1ms sleep time - we will replace this when we move to a work item

		// Only run if vehicle_attitude is updated
		// Note - we use this as if it hasn't been updated after startup then quaternion will be zero
		// Hence the maths generates NaNs
		if (_vehicle_attitude_sub.updated())
		{

			//Update data
			_vehicle_attitude_sub.update(&_vehicle_attitude);
			_vehicle_attitude_setpoint_sub.update(&_vehicle_attitude_setpoint);
			_hover_thrust_estimate_sub.update(&_hover_thrust_estimate);
			_vehicle_local_position_sub.update(&_vehicle_local_position);

			//Update filter if needed
			hrt_abstime acc_timestamp = _vehicle_attitude_setpoint.timestamp;
			hrt_abstime sample_time = acc_timestamp - _timestamp_prev;
			_timestamp_prev = acc_timestamp;
			if (sample_time > 0) {
				float freq = 1.f / (float(sample_time) * 1e-6f);
				// TODO: this doesn't work well - sample freq jumps around hence 50Hz threshold
				//       might get better when we move to a work item
				if (fabs(_lp_filter.get_sample_freq() - freq) > 50.0f) {
					_lp_filter.set_cutoff_frequency(freq, 10.f);
					 mavlink_log_info(&_mavlink_log_pub, "filter reset to %f Hz", double(freq));
				}
				// mavlink_log_info(&_mavlink_log_pub, "sample time %f", double(freq));
			}

			// Body attitude
			const float &qw = _vehicle_attitude.q[0];
			const float &qx = _vehicle_attitude.q[1];
			const float &qy = _vehicle_attitude.q[2];
			const float &qz = _vehicle_attitude.q[3];
			Quatf att_quat(qw, qx, qy, qz);

			// Acceleration from accelerometer
			// (from Richard - Use vehicle_local_position if you want the EKF acceleration output in the body frame)
			const float &ax = _vehicle_local_position.ax;
			const float &ay = _vehicle_local_position.ay;
			const float &az = _vehicle_local_position.az;
			Vector3f acc_measured_body(ax, ay, az);
			Vector3f acc_measured = att_quat.conjugate(acc_measured_body);

			// Expected acceleration from thrust (note thrust_body[1] and thrust_body[2] will be zero)
			const float hover_thrust = math::max(0.1f, _hover_thrust_estimate.hover_thrust);
			const float &acc_expected_x = _vehicle_attitude_setpoint.thrust_body[0] * 9.81f / hover_thrust;
			const float &acc_expected_y = _vehicle_attitude_setpoint.thrust_body[1] * 9.81f / hover_thrust;
			const float &acc_expected_z = _vehicle_attitude_setpoint.thrust_body[2] * 9.81f / hover_thrust;
			Vector3f acc_expected_body(acc_expected_x, acc_expected_y, acc_expected_z);
			Vector3f acc_expected = att_quat.conjugate(acc_expected_body);

			// Subtract expected acceleration from measured acceleration to estimate drag acceleration.
			// (assuming measured = expected + drag)
			Vector3f drag_acc = acc_measured - acc_expected; // * 0.8f // - scale acc_expected by 0.8 to test this in jmavsim

			// Filter the drag acceleration
			Vector3f drag_acc_filtered = _lp_filter.apply(drag_acc);

			// Convert drag_acceleration_filtered back into body fram
			Vector3f drag_acc_filtered_body = att_quat.conjugate_inversed(drag_acc_filtered);

			// Cross product with Centre of Pressure (CoP) offset z to get moment acting on Centre of Gravity (CoG) in body frame.
			// We use a fixed 0.1m offset above the CoG
			// This can then be scaled accordingly with the gain in the rate controller
			Vector3f drag_acc_moment_body = drag_acc_filtered_body.cross(Vector3f(0.f, 0.f, -0.1f));

			// Populate drag for publishing
			drag_estimator_s drag{};
			acc_measured.copyTo(drag.acc_measured);
			acc_expected.copyTo(drag.acc_expected);
			drag_acc.copyTo(drag.drag_acc);
			drag_acc_filtered.copyTo(drag.drag_acc_filtered);
			drag_acc_filtered_body.copyTo(drag.drag_acc_filtered_body);
			drag_acc_moment_body.copyTo(drag.drag_acceleration_moment_body);
			drag.timestamp = hrt_absolute_time();
			_drag_estimator_pub.publish(drag);

			// mavlink_log_info(&_mavlink_log_pub, "acc ekf 0 = %f", double(ax));

			parameters_update();




			// PULL timestamps from subs, find difference, 1/ them and use that as freq.
			// Initialise at 100Hz and if this changes by 3-5Hz ish then reset the filter.


			// TODO: Delete sandbox below
/*
			// Take attitude in quaternions


			// Take hover thrust estimate to quantify relationship between thrust and acceleration.
			// i.e hover thrust should be equal to g
			thrust_coeff = float(9.81)/_hover_thrust_estimate.hover_thrust;

			// Apply thrust_coeff to drone thrust to get acceleration
			acc_fwd = _vehicle_attitude_setpoint.thrust_body[0] * thrust_coeff;
			acc_right = _vehicle_attitude_setpoint.thrust_body[1] * thrust_coeff;
			acc_down = _vehicle_attitude_setpoint.thrust_body[2] * thrust_coeff;
			matrix::Vector3f acc(acc_fwd, acc_right, acc_down);

			matrix::Quatf quat(_vehicle_attitude.q);
			//matrix::Vector3f vec2 = quat.conjugate(acc);

			matrix::Eulerf euler(quat);


			float roll = AttQuatToRoll(qw, qx, qy, qz);


			//mavlink_log_info(&_mavlink_log_pub, "quat0 = %f, roll = %f", double(quat(0)), double(roll));
			//mavlink_log_info(&_mavlink_log_pub, "vec2(0) = %f, vec2(1) = %f, vec2(2) = %f", double(vec2(0)), double(vec2(1)), double(vec2(2)));
*/




		}

	}
}

void DragEstimator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}


float DragEstimator::AttQuatToRoll(float _qw, float _qx, float _qy, float _qz) {

	float x1 = 2 * (_qw * _qx + _qy * _qz);
	float x2 = 1 - 2 * (pow(_qx, 2) + pow(_qy, 2));

	return matrix::atan2(x1, x2) * 180.0f/3.14159f;

}



int DragEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("DragEstimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int drag_estimator_main(int argc, char *argv[])
{
	return DragEstimator::main(argc, argv);
}
