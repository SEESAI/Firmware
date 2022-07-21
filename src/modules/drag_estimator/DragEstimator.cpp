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
				      1024,
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
	// Example: run the loop synchronized to the vehicle_attitude topic publication
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_hover_thrust_estimate_sub = orb_subscribe(ORB_ID(hover_thrust_estimate));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));



	px4_pollfd_struct_t fds[4];
	fds[0].fd = _vehicle_attitude_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_attitude_setpoint_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _hover_thrust_estimate_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _sensor_combined_sub;
	fds[3].events = POLLIN;

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN || fds[1].revents & POLLIN || fds[2].revents & POLLIN) {

			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
			orb_copy(ORB_ID(vehicle_attitude_setpoint), _vehicle_attitude_setpoint_sub, &_vehicle_attitude_setpoint);
			orb_copy(ORB_ID(hover_thrust_estimate), _hover_thrust_estimate_sub, &_hover_thrust_estimate);
			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

			// Acceleration from accelerometer
			float ax = _sensor_combined.accelerometer_m_s2[0];
			float ay = _sensor_combined.accelerometer_m_s2[1];
			float az = _sensor_combined.accelerometer_m_s2[2];

			// TODO: maths


			drag_estimator_s drag;
			// TODO: populate drag for publishing


			_drag_estimator_pub.publish(drag);




			// TODO: do something with the data...
/*
			// Take attitude in quaternions
			qw = _vehicle_attitude.q[0];
			qx = _vehicle_attitude.q[1];
			qy = _vehicle_attitude.q[2];
			qz = _vehicle_attitude.q[3];

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
*/










			mavlink_log_info(&_mavlink_log_pub, "quat0 = %f, roll = %f", double(quat(0)), double(roll));
			//mavlink_log_info(&_mavlink_log_pub, "vec2(0) = %f, vec2(1) = %f, vec2(2) = %f", double(vec2(0)), double(vec2(1)), double(vec2(2)));





		}

	}

	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_vehicle_attitude_setpoint_sub);
	orb_unsubscribe(_hover_thrust_estimate_sub);
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
