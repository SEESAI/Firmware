# Joystick Yaw Acceleration Limit

| | |
|---|---|
| **Status** | Since v1.13 (#89). Proposed port to v1.17. |
| **Code** | `StickYaw::generateYawSetpoint()` in `src/lib/stick_yaw/StickYaw.cpp` |

## Summary

Upstream PX4 does not limit how quickly the joystick can change the commanded yaw rate.

On Yealm a large joystick yaw input asks for more yaw torque than the motors can produce.

This patch adds a limiter on the joystick yaw rate setpoint, which caps the commanded yaw acceleration.

## Why

### Upstream shapes the joystick with a low-pass filter

v1.17 passes the joystick yaw rate through a first-order low-pass filter ($\tau$ = `MPC_MAN_Y_TAU` = 0.08 s).

It does not limit the yaw acceleration, letting it grow with the size of the joystick movements.

v1.13 hard-coded a 360 deg/s² limit instead.

PR #89 made it configurable (set to 180 deg/s² on Yealm) because 360 deg/s² was still too high for Yealm.

### Why not just increase the filter's time constant?

Increasing the filter's time constant would limit the peak yaw acceleration, but would make joystick yaw control too sluggish for effective flights.

## Design

### Limit yaw acceleration

We use `SlewRate<float>` from `src/lib/slew_rate/SlewRate.hpp` to limit the rate of change of the target yaw rate.

Each step it moves the yaw rate toward the target by at most `MPC_MAN_Y_ACC` $\cdot\ dt$.

`MPC_MAN_Y_ACC` $= 0$ (default) bypasses the limiter, so other airframes behave exactly as upstream.

On Yealm we set `MPC_MAN_Y_TAU` $= 0$ which disables the low pass filter. This leaves only the yaw rate limiter matching the behaviour on v1.13 in Position mode. (`SYS_VEHICLE_RESP` must stay negative, which is the default).

Stabilized and Altitude mode now also get the limit (on v1.13 they did not). This is intentional, because the motor saturation risk is the same in every mode.

## Notes for porting new PX4 Versions

v1.13 implemented this by making the existing `SlewRate` in `StickYaw` configurable.

v1.17 replaced that `SlewRate` with an `AlphaFilter`, so the limiter is re-added alongside the filter.

The v1.13 parameter `MC_YAW_ACC_MAX` is renamed to `MPC_MAN_Y_ACC` in v1.17, to sit with the other joystick yaw parameters. Update Yealm's parameter file, as the old name is no longer recognised.

When in offboard mode, yaw setpoints do not pass through `StickYaw` and are not affected by this patch.
