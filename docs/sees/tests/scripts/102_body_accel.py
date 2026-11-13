#!/usr/bin/env python3
"""
Test 102: offboard body-frame (FRDY) acceleration.

Flies PX4 SITL forward and back with the acceleration commands SI2 sends.
Watch the result in QGC, see ../test_102.md.
"""
import math
import time

from pymavlink import mavutil

mav = mavutil.mavlink

PX4_URL = "udpin:0.0.0.0:14540"
TAKEOFF_ALT = 20.0  # m
ACCEL = 3.0         # m/s², forward then backward (braking)
PULSE_TIME = 3.0    # s
PAUSE_TIME = 2.0    # s
REPEATS = 3
SEND_RATE = 20      # Hz

PX4_MODE_OFFBOARD = 6

# Like SI2: use acceleration and yaw rate, ignore everything else.
ACCEL_AND_YAW_RATE_ONLY = (
    mav.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


class Drone:
    def __init__(self, url):
        self.link = mavutil.mavlink_connection(url)
        self.link.wait_heartbeat()

    def arm(self):
        while not self._command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            time.sleep(1)  # PX4 refuses until it is ready to fly

    def take_off(self, altitude):
        self._set_param("MIS_TAKEOFF_ALT", altitude)
        self._command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
        self._wait_for_mode("TAKEOFF")
        self._wait_for_mode("LOITER")  # Hold. Offboard sent before this gets overridden.

    def start_offboard(self):
        self.accelerate(0, 1)  # PX4 only enters offboard once setpoints are arriving
        self._command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MODE_OFFBOARD)
        self.accelerate(0, 2)
        if self.link.flightmode != "OFFBOARD":
            raise SystemExit(f"PX4 did not enter offboard (mode is {self.link.flightmode})")

    def accelerate(self, forward, seconds):
        """Command a forward acceleration (FRDY frame) for a number of seconds."""
        end = time.time() + seconds
        while time.time() < end:
            self.link.mav.set_position_target_local_ned_send(
                time_boot_ms=0,
                target_system=self.link.target_system,
                target_component=self.link.target_component,
                coordinate_frame=mav.MAV_FRAME_BODY_NED,
                type_mask=ACCEL_AND_YAW_RATE_ONLY,
                x=0, y=0, z=0,
                vx=0, vy=0, vz=0,
                afx=forward, afy=0, afz=0,
                yaw=0, yaw_rate=0)
            self._read_incoming()
            time.sleep(1 / SEND_RATE)

    def land(self):
        self._command(mav.MAV_CMD_NAV_LAND)

    def _command(self, command, *params):
        """Send a MAVLink command. Returns True if PX4 accepted it."""
        params = list(params) + [0] * (7 - len(params))
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, command, 0, *params)
        ack = self.link.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        return ack is not None and ack.result == mav.MAV_RESULT_ACCEPTED

    def _set_param(self, name, value):
        self.link.mav.param_set_send(self.link.target_system, self.link.target_component,
                                     name.encode(), value, mav.MAV_PARAM_TYPE_REAL32)

    def _wait_for_mode(self, mode):
        while self.link.flightmode != mode:
            self.link.recv_match(type="HEARTBEAT", blocking=True)

    def _read_incoming(self):
        """Read everything PX4 has sent, which keeps link.flightmode up to date."""
        while self.link.recv_msg():
            pass


def main():
    print("Connecting to PX4 SITL")
    drone = Drone(PX4_URL)

    print(f"Taking off to {TAKEOFF_ALT:.0f} m")
    drone.arm()
    drone.take_off(TAKEOFF_ALT)

    print("Switching to offboard")
    drone.start_offboard()

    for i in range(1, REPEATS + 1):
        print(f"Pulse {i}/{REPEATS}: forward, then brake")
        drone.accelerate(+ACCEL, PULSE_TIME)
        drone.accelerate(-ACCEL, PULSE_TIME)
        drone.accelerate(0, PAUSE_TIME)

    print("Landing")
    drone.land()


if __name__ == "__main__":
    main()
