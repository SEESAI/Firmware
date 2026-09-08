# Offboard Body-Frame Acceleration: Yaw-Only Rotation

| | |
|---|---|
| **Status** | Since v1.13 ([#84](https://github.com/SEESAI/Firmware/pull/84)), ported to v1.17 ([#102](https://github.com/SEESAI/Firmware/pull/102)) |
| **Code** | `handle_message_set_position_target_local_ned()` in `src/modules/mavlink/mavlink_receiver.cpp` |

## The Issue

Currently PX4 treats offboard acceleration as if it's in the **FRD** frame.

SI2 outputs acceleration commands in the **FRDY** frame. This is a drone's FRD frame but with roll = pitch = 0.

This means when the drone is tilted (e.g. braking), part of a horizontal acceleration command becomes a vertical one.

This patch rotates offboard acceleration by yaw only, as PX4 already does for velocity.

## Why

SI2 has its own pose estimate, which does not depend on PX4. In fact the two might disagree.

Therefore we need to send commands that do not depend on SI2 & PX4 sharing the same world frame.

The FRDY frame allows us to do this.

As SI2 and PX4 each convert using their own heading, it doesn't matter if they disagree about where north is.

## Deviation from MAVLink Spec

SI2 sends acceleration commands via the deprecated `MAV_FRAME_BODY_NED` MAVLink frame.

This frame is defined as NED for position and FRD for velocity and acceleration.

For body frame setpoints, PX4 only accepts offboard commands in the frame `MAV_FRAME_BODY_NED`.

Therefore this patch deviates from the MAVLink spec (acceleration now in FRDY instead of FRD).

Upstream PX4 is also inconsistent with MAVLink (velocity in FRDY, acceleration in FRD).

## Notes for porting new PX4 Versions

MAVLink has `MAV_FRAME_LOCAL_FRD` which is the correct MAVLink frame for FRDY. PX4 does not yet accept it.
