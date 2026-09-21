> Note: This is an example. Not a real test

**Test ID** | 101
--|--
**PR** | [101](https://github.com/SEESAI/Firmware/pull/101)
**Requirement** | No unexpected yaw on takeoff or hold with a small `NAV_ACC_RAD`
**Test lane** | SITL

**Background**
- With `MPC_YAW_MODE` 0 (default), `FlightTaskAuto::_set_heading_from_mode()` points the nose at the target whenever horizontal distance to it exceeds the acceptance radius. During takeoff and hold the target is directly overhead, so a tiny radius makes the vehicle chase its own position jitter. The fix floors the radius used for this check at 1 m (`MIN_HEADING_UPDATE_RADIUS`).
- SITL reproduces the bug: SIH hover drift (~0.7-0.8 m) is far above 0.05 m. A flight test is not needed to verify the fix.

**Test Method**
- `make px4_sitl sihsim_quadx`, wait for the EKF to have a valid local position.
- `param set NAV_ACC_RAD 0.05` (the parameter's min).
- `commander takeoff`, then hold for ~40 s. Sample `vehicle_local_position` (`heading`, `x`, `y`) at 2 Hz, or review the ulog.
- Repeat with `param set NAV_ACC_RAD 2` (multicopter default).
- Negative control (optional, to confirm the test still detects the bug): set `MIN_HEADING_UPDATE_RADIUS` to `0.f`, rebuild, repeat at 0.05.

**Pass criteria**
- heading stays within ±10° of its value at takeoff;
- `param show NAV_ACC_RAD` still reads back the value that was set (the fix doesn't rewrite the param).

**Results**

Date: 2026/09/10

Heading change vs takeoff, cumulative yaw travel, max horizontal drift

Build | `NAV_ACC_RAD` | Heading range | Yaw travel | Max drift | Result
--|--|--|--|--|--
Fix | 0.05 | -2.1° to +2.4° | 29° | 0.82 m | Pass
Fix | 2 | -2.5° to +1.5° | 32° | 0.84 m | Pass
Negative control (floor = 0) | 0.05 | -146° to +112° | 761° | 0.73 m | Fails as expected

**Note:** SIH drift (~0.8 m) is close to the 1 m floor. Real wind/GNSS drift above 1 m would make the vehicle turn toward the takeoff point. That is the same behaviour as upstream with `NAV_ACC_RAD` 1, but it may justify a larger floor (e.g. 2 m, the MC default). Check in flight logs from Whitby.
