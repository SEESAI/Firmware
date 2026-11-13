#!/usr/bin/env python3
"""
Test 124: magnetometer 50 Hz filter.

Acts as PX4's simulator for a drone sitting still on the ground,
whose magnetometer picks up 50 Hz power-line noise.
Watch the result in QGC, see ../test_124.md.

    Terminal 1:  make px4_sitl none_iris
    Terminal 2:  python3 docs/sees/tests/scripts/124_mag_noise.py
"""
import itertools
import math
import time

from pymavlink import mavutil

mav = mavutil.mavlink

EARTH_FIELD = (0.21, 0.0, 0.43)  # G, forward/right/down. The drone is level and faces north.
NOISE = 0.3                      # G, amplitude of the power-line noise, added to forward
POWER_LINE = 50                  # Hz
STEP = 0.002                     # s, sensors at 500 Hz
MAG_EVERY = 3                    # magnetometer every 3rd step: 6 ms, ~167 Hz like the IST8310
GRAVITY = 9.81                   # m/s²

IMU = (mav.HIL_SENSOR_UPDATED_XACC | mav.HIL_SENSOR_UPDATED_YACC | mav.HIL_SENSOR_UPDATED_ZACC
       | mav.HIL_SENSOR_UPDATED_XGYRO | mav.HIL_SENSOR_UPDATED_YGYRO | mav.HIL_SENSOR_UPDATED_ZGYRO)
BARO = mav.HIL_SENSOR_UPDATED_ABS_PRESSURE | mav.HIL_SENSOR_UPDATED_PRESSURE_ALT | mav.HIL_SENSOR_UPDATED_TEMPERATURE
MAG = mav.HIL_SENSOR_UPDATED_XMAG | mav.HIL_SENSOR_UPDATED_YMAG | mav.HIL_SENSOR_UPDATED_ZMAG


def magnetometer(t):
    """Earth's field plus 50 Hz power-line noise, at time t (s)."""
    forward, right, down = EARTH_FIELD
    noise = NOISE * math.sin(2 * math.pi * POWER_LINE * t)
    return forward + noise, right, down


def send_sensors(px4, t, with_mag):
    """Send one sample from a drone sitting still: gravity, no rotation, sea-level pressure."""
    xmag, ymag, zmag = magnetometer(t)
    px4.mav.hil_sensor_send(
        time_usec=round(t * 1e6),
        xacc=0, yacc=0, zacc=-GRAVITY,
        xgyro=0, ygyro=0, zgyro=0,
        xmag=xmag, ymag=ymag, zmag=zmag,
        abs_pressure=1013.25, diff_pressure=0, pressure_alt=0, temperature=20,
        fields_updated=IMU | BARO | (MAG if with_mag else 0),
        id=0)


def main():
    print("Waiting for PX4 (make px4_sitl none_iris)")
    px4 = mavutil.mavlink_connection("tcpin:0.0.0.0:4560")
    px4.wait_heartbeat()
    print(f"Connected. Adding {NOISE} G of {POWER_LINE} Hz noise to the magnetometer. Ctrl-C to stop.")

    # PX4 runs in lockstep with the timestamps we send, so its clock stops when we stop.
    start = time.time()
    for step in itertools.count(1):
        t = step * STEP
        send_sensors(px4, t, with_mag=(step % MAG_EVERY == 0))
        while px4.recv_msg():  # PX4 replies with motor outputs, which we ignore
            pass
        time.sleep(max(0.0, start + t - time.time()))  # keep to real time


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
