**Test ID** | 120
--|--
**PR** | [120](https://github.com/SEESAI/Firmware/pull/120) (port of v1.13 [#89](https://github.com/SEESAI/Firmware/pull/89))
**Requirement** | Joystick yaw acceleration never exceeds `MPC_MAN_Y_ACC`
**Test lane** | SITL

**Background**
- Upstream v1.17 only low-pass filters the joystick yaw rate, so peak yaw acceleration grows with the stick step. On Yealm this asks for more yaw torque than the motors can produce. The patch adds a yaw acceleration limit (`MPC_MAN_Y_ACC`, 180 deg/s² on Yealm).
- This test uses a deliberately low limit (30 deg/s²) so the ramp is slow enough to see by eye. The limiter works the same at any value.

**Test Method**
- `make px4_sitl sihsim_quadx`, connect QGC and enable a joystick (gamepad or on-screen).
- In QGC, open Analyze Tools > MAVLink Inspector and chart `ATTITUDE.yawspeed` (rad/s).
- `param set MPC_MAN_Y_ACC 30`, take off, switch to Position mode.
- Snap the yaw stick to full right, hold until the yaw rate stops rising, release. Screenshot the chart.
- Repeat with `param set MPC_MAN_Y_ACC 0` (negative control: confirms the stick was moved fast enough).

**Pass criteria**

Full yaw rate is `MPC_MAN_Y_MAX` = 150 deg/s ≈ 2.6 rad/s.

`MPC_MAN_Y_ACC` | Time to full yaw rate | What you'll see
--|--|--
30 | 4-6 s (expected 150 / 30 = 5 s) | Slow, straight ramp. Keeps turning ~1 full turn after release.
0 | < 1 s | Near-instant jump. Stops quickly after release.

**Results**

Not yet run.

`MPC_MAN_Y_ACC` | Time to full yaw rate | Screenshot | Result
--|--|--|--
30 | | |
0 | | |
