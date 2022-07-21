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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/drag_estimator.h>
#include <uORB/Publication.hpp>
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>


extern "C" __EXPORT int drag_estimator_main(int argc, char *argv[]);


class DragEstimator : public ModuleBase<DragEstimator>, public ModuleParams
{
public:
	DragEstimator(int example_param, bool example_flag);

	virtual ~DragEstimator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static DragEstimator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	float AttQuatToRoll(float _qw, float _qx, float _qy, float _qz);

	// Quaternion values from vehicle_attitude
	float qw{0};
	float qx{0};
	float qy{0};
	float qz{0};

	float acc_fwd{0};
	float acc_right{0};
	float acc_down{0};

	// Hover thrust estimate from hover_thrust_estimate
	float thrust_coeff{0};


private:

	//Subscriptions

	int _vehicle_attitude_sub {-1};
	int _vehicle_attitude_setpoint_sub {-1};
	int _hover_thrust_estimate_sub {-1};

	vehicle_attitude_s		_vehicle_attitude{};
	vehicle_attitude_setpoint_s	_vehicle_attitude_setpoint{};
	hover_thrust_estimate_s		_hover_thrust_estimate;

	uORB::Publication<drag_estimator_s>	_drag_estimator_pub{ORB_ID(drag_estimator)};

	orb_advert_t           _mavlink_log_pub{nullptr};





	//uORB::Publication<vehicle_attitude_s>		_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	//uORB::Publication<vehicle_attitude_setpoint_s>		_vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	//uORB::Publication<hover_thrust_estimate_s>		_hover_thrust_estimate{ORB_ID(hover_thrust_estimate)};
};

