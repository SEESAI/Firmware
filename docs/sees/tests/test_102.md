**Test ID** | 102
--|--
**PR** | [102](https://github.com/SEESAI/Firmware/pull/102) (port of v1.13 [#84](https://github.com/SEESAI/Firmware/pull/84))
**Requirement** | When the drone tilts, PX4 does not turn SI2's forward/backward acceleration into vertical acceleration
**Test lane** | SITL

**Background**
- SI2 sends acceleration commands relative to the drone's heading. Upstream PX4 also applies the drone's tilt to them. So when the drone pitches, part of a forward or backward command becomes a downward one, and the drone sinks. The patch ignores tilt and only uses heading.
- Braking in SI2 Joystick mode tilts the drone a lot, so a large part of the horizontal command becomes vertical.

**Test Method**
- Start PX4 SITL: `make px4_sitl sihsim_quadx` (no SI2 or AirSim).
- In QGC, open Analyze Tools > MAVLink Inspector > `POSITION_TARGET_LOCAL_NED` and chart `afx` and `afz` (positive = down).
- Run [`scripts/102_body_accel.py`](./scripts/102_body_accel.py). It takes off to 20 m, switches to offboard and sends the same acceleration commands as SI2: 3 × (3 m/s² forward for 3 s, then 3 m/s² backward for 3 s), with 0 vertical. Then it lands.
- Screenshot the chart.
- Remove the patch (commit `6b84e02196`), rebuild and repeat.

**Pass criteria**

Each `afx` pulse tilts the drone about 17°.

Build | Expected chart
--|--
With patch | `afx` pulses ±3 m/s². `afz` stays at 0.
Without patch | `afx` pulses about ±2.9 m/s². `afz` sits at about +0.85 m/s² (downward) during each pulse, with a brief upward spike when forward switches to brake.

Pass if `afz` stays at 0 with the patch.

**Results**

Build | `afz` flat while tilted? | Screenshot | Result
--|--|--|--
With patch | yes | attached | pass
Without patch | no | attached | fail
