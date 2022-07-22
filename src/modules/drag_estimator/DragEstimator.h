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
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/drag_estimator.h>
#include <uORB/Publication.hpp>
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using namespace matrix;


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


	float acc_fwd{0};
	float acc_right{0};
	float acc_down{0};

	// Hover thrust estimate from hover_thrust_estimate
	float thrust_coeff{0};


private:
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

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

	vehicle_local_position_s	_vehicle_local_position{};
	vehicle_attitude_s		_vehicle_attitude{};
	vehicle_attitude_setpoint_s	_vehicle_attitude_setpoint{};
	hover_thrust_estimate_s		_hover_thrust_estimate{};

	math::LowPassFilter2p<matrix::Vector3f> _lp_filter{100.f, 10.f};
	hrt_abstime _timestamp_prev{0};

	uORB::Publication<drag_estimator_s>	_drag_estimator_pub{ORB_ID(drag_estimator)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};

	orb_advert_t           _mavlink_log_pub{nullptr};





	//uORB::Publication<vehicle_attitude_s>		_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	//uORB::Publication<vehicle_attitude_setpoint_s>		_vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	//uORB::Publication<hover_thrust_estimate_s>		_hover_thrust_estimate{ORB_ID(hover_thrust_estimate)};
};

