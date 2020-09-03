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
/*

sees.ai - Eduardo Aldaz-Carroll - 1 Sep 2020

We want to be able to have the Observer (OB) be able to control the drone while it is also controlled remotely by a joystick
connected to QGC.

QGC reads the joystick and uses the info to send MANUAL_CONTROL messages, it fills axis and buttons from joytick.
PX4 takes the MANUAL_CONTROL messages and converts them into an instance of manual_control_setpoint uorb messages.
It also stores the information it receives from the RC Tx into another instance manual_control_setpoint uorb messages.
manual_control_setpoint is a multinstance message.
All the modules that subscribe to this message connect to the first instance that is registered.


We have enabled a switch on the RC Tx (unusued VTOL transition_switch) that the OB can toggle to gain/release control of the drone.
Also all the mode switches and the kill switch from the RC remain operational even if Joystick is controlling.
The RP can gain/release control by using button A from the Joystick.

To implement this we have developped a module called ob_manual_control that selects the RC Tx or Mavlink manual control messages and forwards them on.
This is done by creating two new types of topic, manual_control_setpoint_rc and manual_control_setpoint_mav created by the RC and mavlink respectively ,
previously they both used multi topic standard_manual_control.

Now the ob_manual_control module select one originator (RC/Mav) depending on the switch and forwards it on as the standard manual_control_Setpoint messages.

BThe variable buttons of MANUAL_CONTROL, bits 12,13 contain who is actually in control (RC =1 or 2-5 Mavlink instance).

Have also added the Status of comms sent through VEHICLE_STATUS error_count1, error_count2, etc

Note: The switches on the RC Tx are mapped to manual_control_setpoint individually, then some of them are bundled as bit mask inside MANUAL_CONTROL buttons.
	QGC maps the buttons of the joystick as expected onto the MANUAL_CONTROL message. However PX4 ignores them.
*/



#include "OBManualControl.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>

int OBManualControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int OBManualControl::custom_command(int argc, char *argv[])
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


int OBManualControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ob_manual_command",
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

OBManualControl *OBManualControl::instantiate(int argc, char *argv[])
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

	OBManualControl *instance = new OBManualControl(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;

}

OBManualControl::OBManualControl(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void OBManualControl::run()
{
	// Run the loop synchronized to the manual_control_setpoint topic publication
	_manual_control_sub_mav = orb_subscribe(ORB_ID(manual_control_setpoint_mav));
	_manual_control_sub_rc = orb_subscribe(ORB_ID(manual_control_setpoint_rc));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _manual_control_sub_mav;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_sub_mav;
	fds[1].events = POLLIN;

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

		} else if (fds[0].revents & POLLIN || fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(manual_control_setpoint_rc), _manual_control_sub_rc, &_manual_control_setpoint_rc);
			orb_copy(ORB_ID(manual_control_setpoint_mav), _manual_control_sub_mav, &_manual_control_setpoint_mav);

			bool switch_toggled = SwitchToggled(&_manual_control_setpoint_rc, &_manual_control_setpoint_mav);

			switch (_state) { // State is indirectly shown on manual_control_setpoint.data_source . 1 For RC , 2-5 for Mavlink
			case RC_CONTROL: {
					if (switch_toggled) {
						PX4_INFO("Switching to Mav control");
						_state = MAV_CONTROL;
						break; // Exit immediately
					}

					_manual_control_setpoint = _manual_control_setpoint_rc; // First fill with rc as default
					 // The transition_switch is not used in PX4 MC, but set it to zero as an extra precaution
					_manual_control_setpoint.transition_switch = 0; // Used to toggle from RC to Joystick

					_manual_control_sub.publish(_manual_control_setpoint);
					break;
				}

			case MAV_CONTROL: {
					if (switch_toggled) {
						PX4_INFO("Switching to RC control");
						_state = RC_CONTROL;
						break; // Exit immediately
					}
					_manual_control_setpoint = _manual_control_setpoint_mav; // First fill with mav as default

					UseRCSetpoints(&_manual_control_setpoint_rc, &_manual_control_setpoint); // Modify those values that are a combination
					// The transition_switch is not used in PX4 MC, but set it to zero as an extra precaution
					_manual_control_setpoint.transition_switch = 0; // Used to toggle from RC to Joystick

					_manual_control_sub.publish(_manual_control_setpoint);
					break;
				}
			}
		}
	}

	//_manual_control_sub_mav.unsubscribe();
	orb_unsubscribe(_manual_control_sub_mav);
	orb_unsubscribe(_manual_control_sub_rc);
}


// This functions incorporates the switches from RC into the Joystick message.
// When usign the Joystick the functions that these RC switches provides are not done through manual_control_setpoint messages but
// directly through appropiate mavlink commnads from QGC
void OBManualControl::UseRCSetpoints(manual_control_setpoint_s *manual_control_setpoint_rc, manual_control_setpoint_s *manual_control_setpoint)
{
	// Signal coming from both
	manual_control_setpoint->kill_switch = manual_control_setpoint_rc->kill_switch;

	// Joystick (mav) doesn't set any of the mode switches in manual_control, it's QGC that sends the necessery modes directly
	// through the relevant mavlink message. So use those of the RC instead so that RC can control modes even when joystick in control.
	manual_control_setpoint->mode_switch = manual_control_setpoint_rc->mode_switch;
	if(manual_control_setpoint->mode_switch == 0){
		// Mode selection done  by a single channel
		manual_control_setpoint->mode_slot = manual_control_setpoint_rc->mode_slot;    // This switch is used when setting the mode with a single channel
	}
	else{
		PX4_INFO("Multi Channel mode");
		// Mode selection set by multiple Channels, only interested in these:
		manual_control_setpoint->return_switch = manual_control_setpoint_rc->return_switch;
		manual_control_setpoint->loiter_switch = manual_control_setpoint_rc->loiter_switch;
		// There is no switch dedicated to Land for some reason !
	}
}

// The joystick buttons in PX4 are not used, so had to hardcode the transition switch to first button (A) in the controller in
// mavlink_receiver.cpp.
// This function detects if the switch has toggled in either joystick or RC Tx
bool OBManualControl::SwitchToggled(manual_control_setpoint_s *manual_control_setpoint_rc,
				    manual_control_setpoint_s *manual_control_setpoint_mav)
{
	static bool first_run = true;
	static uint8_t
	transition_switch_rc_prev;   // We use transition_switch as it's only used for VTOL.
	static uint8_t transition_switch_mav_prev;
	bool toggled_rc = false;
	bool toggled_mav = false;
	bool toggled = false;

	if (first_run) {
		transition_switch_rc_prev = manual_control_setpoint_rc->transition_switch;
		transition_switch_mav_prev = manual_control_setpoint_mav->transition_switch;
		first_run = false;
	}

	toggled_rc = transition_switch_rc_prev ^ manual_control_setpoint_rc->transition_switch; // Xor => Output != 0 if they are different

	if (transition_switch_mav_prev == 0
	    && manual_control_setpoint_mav->transition_switch == 1) { // Joystick is momentary button default value 0, so only looking for transition from 0 to 1
		toggled_mav = true;

	} else {
		toggled_mav = false;
	}

	toggled = toggled_rc | toggled_mav ; // true if any of them toggle

	//PX4_INFO("toggled value %d", toggled);

	transition_switch_rc_prev = manual_control_setpoint_rc->transition_switch;
	transition_switch_mav_prev = manual_control_setpoint_mav->transition_switch;


	return (toggled);
}

int OBManualControl::print_usage(const char *reason)
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
$ module start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("OBManualControl", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ob_manual_control_main(int argc, char *argv[])
{
	return OBManualControl::main(argc, argv);
}
