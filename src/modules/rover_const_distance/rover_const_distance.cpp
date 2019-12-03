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

#include "rover_const_distance.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>



int RoverConstDistance::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int RoverConstDistance::custom_command(int argc, char *argv[])
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


int RoverConstDistance::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rover_const_distance",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
					(px4_main_t)&RoverConstDistance::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

RoverConstDistance *RoverConstDistance::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}


	RoverConstDistance *instance = new RoverConstDistance();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate RoverConstDistance object");
	}

	return instance;
}

RoverConstDistance::RoverConstDistance()
	: ModuleParams(nullptr), _px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, distance_sensor_s::ROTATION_DOWNWARD_FACING)
{
	_px4_rangefinder.set_min_distance(0);
	_px4_rangefinder.set_max_distance(0.5);
	_px4_rangefinder.set_fov(0.008); // Divergence 8 mRadian


}

void RoverConstDistance::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

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

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			//I'm using sensor_combined as a hacky source of timer, not sure what speed the
			// distance sensor is expected to run at so running at same speed as other sensors

			//_distance_sensor.

			const hrt_abstime timestamp_sample = hrt_absolute_time();
			float distance_m = 0.1;
			int8_t signal_quality = 10;
			_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);
			/*
			if (_const_distance_pub != nullptr) {
				orb_publish(ORB_ID(distance_sensor), _const_distance_pub, &_distance_sensor);

			} else {
				_const_distance_pub = orb_advertise(ORB_ID(distance_sensor), &_distance_sensor);
			}
			*/
			//PX4_INFO("Running \n");

		}

		//

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void RoverConstDistance::parameters_update(bool force)
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

int RoverConstDistance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a simulated distance reading which is fixed to D m  to be used by rovers, so that
they don't require a distance sensor such as a Lidar Lite

### Implementation
THe module publishes distance readings at a fixed rate of T secs.

### Examples
CLI usage example:
$ rover_const_distance start
$ rover_const_distance status
$ rover_const_distance stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_const_distance", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_const_distance_main(int argc, char *argv[])
{
	return RoverConstDistance::main(argc, argv);
}
