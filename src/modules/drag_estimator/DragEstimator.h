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

#pragma once


#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/drag_estimator.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/Publication.hpp>
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using namespace matrix;
using namespace time_literals;


class DragEstimator : public ModuleBase<DragEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DragEstimator();
	~DragEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	float AttQuatToRoll(float _qw, float _qx, float _qy, float _qz);


private:
	void Run() override;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// TODO: remove unwanted params, set cop offset
	DEFINE_PARAMETERS(
		//(ParamFloat<px4::params::COP_OFFSET_Z) _param_cop_offset_z,
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)


	//Subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _sensor_combined_sub{ ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};

	vehicle_local_position_s	_vehicle_local_position{};
	vehicle_attitude_s		_vehicle_attitude{};
	vehicle_attitude_setpoint_s	_vehicle_attitude_setpoint{};
	vehicle_land_detected_s		_vehicle_land_detected{};
	sensor_combined_s		_sensor_combined{};
	vehicle_acceleration_s		_vehicle_acceleration{};

	float _hover_thrust{0.5};

	bool _landed{true};
	bool _maybe_landed{true};

	math::LowPassFilter2p<matrix::Vector3f> _lp_filter{100.f, 10.f};
	hrt_abstime _timestamp_prev{0};

	uORB::Publication<drag_estimator_s>	_drag_estimator_pub{ORB_ID(drag_estimator)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};

	orb_advert_t           _mavlink_log_pub{nullptr};


	bool _armed{false};
};


extern "C" __EXPORT int drag_estimator_main(int argc, char *argv[]);

