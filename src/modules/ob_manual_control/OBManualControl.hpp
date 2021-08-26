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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/PublicationMulti.hpp>
#include <systemlib/mavlink_log.h>

extern "C" __EXPORT int ob_manual_control_main(int argc, char *argv[]);


class OBManualControl : public ModuleBase<OBManualControl>, public ModuleParams
{
public:
	OBManualControl(int example_param, bool example_flag);

	virtual ~OBManualControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static OBManualControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool SwitchToggled(manual_control_switches_s *manual_control_switches_rc, manual_control_setpoint_s *manual_control_setpoint_rc,
			   manual_control_switches_s *manual_control_switches_mav, manual_control_setpoint_s *manual_control_setpoint_mav);
	void UseRCSetpoints(manual_control_switches_s *manual_control_switches_rc, manual_control_setpoint_s *manual_control_setpoint_rc,
			   manual_control_switches_s *manual_control_switches);

private:

	// Subscriptions
	//uORB::Subscription	_manual_control_sub_mav{ORB_ID(manual_control_setpoint_mav)};
	//uORB::Subscription	_manual_control_sub_rc{ORB_ID(manual_control_setpoint_rc)};
	int _manual_control_switches_sub_mav{-1};		/**< notification of manual control switches updates from MAVLink */
	int _manual_control_switches_sub_rc{-1};		/**< notification of manual control switches updates from RC transmitter */
	int _manual_control_setpoint_sub_mav{-1};               /**< notification of manual control setpoint updates from MAVLink */
	int _manual_control_setpoint_sub_rc{-1};                /**< notification of manual control setpoint updates from RC transmitter */

	//uORB::Subscription	_manual_control_sub_rc{ORB_ID(manual_control_setpoint_rc)};
	manual_control_setpoint_s       _manual_control_setpoint{};
	manual_control_setpoint_s	_manual_control_setpoint_rc{};
	manual_control_setpoint_s	_manual_control_setpoint_mav{};
	manual_control_switches_s	_manual_control_switches{};
	manual_control_switches_s       _manual_control_switches_rc{};
	manual_control_switches_s       _manual_control_switches_mav{};

	uORB::PublicationMulti<manual_control_switches_s> 	_manual_control_switches_sub{ORB_ID(manual_control_switches)}; /**< notification of manual control switch updates coming from MAVLink */
	uORB::PublicationMulti<manual_control_setpoint_s>       _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; /**< notification of manual control setpoint updates coming from MAVLink */


	enum State {RC_CONTROL, MAV_CONTROL} _state{MAV_CONTROL};

	orb_advert_t           _mavlink_log_pub{nullptr};



};
